"""Language -> plan. Two layers:

* rules : deterministic keyword parser (always available, also used as guard)
* vlm   : small vision-language model (llama.cpp server, OpenAI-compatible API)
          sees the camera image + object list and answers with schema-constrained
          JSON, so even a tiny model can only emit valid actions / object ids.

Plan = list of Step(action, object, target); action in
pick_place | home | say.
"""
from __future__ import annotations
import base64, io, json, re, time, urllib.request, urllib.error
from dataclasses import dataclass, field
from . import scene as S

COLORS = {"red": "red_cube", "blue": "blue_cube", "green": "green_cylinder", "yellow": "yellow_box"}
SHAPES = {"cube": ("red_cube", "blue_cube"), "cylinder": ("green_cylinder",), "can": ("green_cylinder",),
          "box": ("yellow_box",), "block": ("yellow_box", "red_cube", "blue_cube")}
ALL_WORDS = r"\b(all|everything|every|each|tidy|clean ?up|clear)\b"
HOME_WORDS = r"\b(home|rest|reset arm|go back|retract|stop moving)\b"
PICK_WORDS = r"\b(put|place|pick|grab|take|move|drop|throw|carry|bring|pack|collect|get|lift|transfer|sort|fetch)\b"
SAY_WORDS = r"^(hi|hello|hey|what|who|how|why|where|which|can you|do you|are you|describe|tell|explain|thanks|thank)"


@dataclass
class Step:
    action: str
    object: str | None = None
    target: str = "basket"
    reply: str = ""


@dataclass
class Plan:
    steps: list = field(default_factory=list)
    reply: str = ""
    source: str = "rules"
    notes: list = field(default_factory=list)
    latency: float = 0.0


def mentioned_objects(text: str):
    t = text.lower()
    found = []
    for c, n in COLORS.items():
        if re.search(rf"\b{c}\b", t) and n not in found:
            found.append(n)
    if not found:
        for w, names in SHAPES.items():
            if re.search(rf"\b{w}s?\b", t):
                found += [n for n in names if n not in found]
    return found


def wants_table(text: str) -> bool:
    return bool(re.search(r"\b(back|table|original|undo)\b", text.lower())) and not re.search(r"\bbasket|bin\b", text.lower())


def rule_plan(text: str, poses: dict) -> Plan:
    t = text.strip().lower()
    p = Plan(source="rules")
    if re.search(HOME_WORDS, t):
        p.steps = [Step("home")]
        p.reply = "Returning the arm to its home pose."
        return p
    outside = [o.name for o in S.OBJECTS if o.name in poses and not S.in_basket(poses[o.name])]
    names = mentioned_objects(t)
    target = "table" if wants_table(t) else "basket"
    if re.search(ALL_WORDS, t) and not names:
        names = outside if target == "basket" else [o.name for o in S.OBJECTS if o.name in poses and S.in_basket(poses[o.name])]
    elif re.search(r"\b(both|all)\b", t) and names:
        pass
    if names and (re.search(PICK_WORDS, t) or re.search(ALL_WORDS, t) or target == "basket"):
        steps = []
        for n in names:
            inside = n in poses and S.in_basket(poses[n])
            if target == "basket" and inside:
                p.notes.append(f"{S.OBJ_BY_NAME[n].label} is already in the basket")
                continue
            if target == "table" and not inside:
                p.notes.append(f"{S.OBJ_BY_NAME[n].label} is already on the table")
                continue
            steps.append(Step("pick_place", n, target))
        p.steps = steps
        p.reply = (f"On it: moving {', '.join(S.OBJ_BY_NAME[s.object].label for s in steps)} to the {target}."
                   if steps else "; ".join(p.notes) or "Nothing to do.")
        return p
    if re.search(ALL_WORDS, t) and not outside:
        p.reply = "Everything is already in the basket."
        return p
    if re.search(r"\b(see|describe|scene|what.s (on|in)|where)\b", t):
        p.steps = [Step("say")]
        p.reply = "I can see: " + describe_scene(poses) + "."
        return p
    p.steps = [Step("say")]
    p.reply = ("I can pick up objects and put them in the basket. Try: 'put the red cube in the basket', "
               "'tidy up', or 'what do you see?'")
    return p


# ---------------------------------------------------------------- VLM client
def _schema():
    return {
        "type": "object",
        "properties": {
            "action": {"type": "string", "enum": ["pick_place", "home", "say"]},
            "object": {"type": "string", "enum": [o.name for o in S.OBJECTS] + ["none"]},
            "target": {"type": "string", "enum": ["basket", "table"]},
            "reply": {"type": "string"},
        },
        "required": ["action", "object", "target", "reply"],
    }


SYSTEM = ("You control a UR5 robot arm above a table. The camera image shows the table, a wooden basket and the objects. "
          "Reply ONLY with JSON: {\"action\": one of pick_place|home|say, \"object\": object id or none, "
          "\"target\": basket|table, \"reply\": short sentence to the user}. "
          "Use pick_place when the user wants an object moved; use say to answer questions about the scene.")


class VLMClient:
    def __init__(self, url: str, timeout: float = 120.0, model: str = "smolvlm"):
        self.url = url.rstrip("/")
        self.timeout = timeout
        self.model = model

    def health(self) -> bool:
        try:
            with urllib.request.urlopen(self.url + "/health", timeout=2) as r:
                return r.status == 200
        except Exception:
            return False

    def ask(self, text: str, jpeg: bytes | None, scene_desc: str) -> dict:
        user = [{"type": "text", "text": f"Objects: {scene_desc}\nUser: {text}"}]
        if jpeg:
            user.insert(0, {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}})
        body = {"model": self.model, "temperature": 0.0, "max_tokens": 160,
                "messages": [{"role": "system", "content": SYSTEM}, {"role": "user", "content": user}],
                "response_format": {"type": "json_object", "schema": _schema()}}
        req = urllib.request.Request(self.url + "/v1/chat/completions", json.dumps(body).encode(),
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            out = json.loads(r.read())
        content = out["choices"][0]["message"]["content"]
        m = re.search(r"\{.*\}", content, re.S)
        return json.loads(m.group(0) if m else content)


def describe_scene(poses: dict) -> str:
    parts = []
    for o in S.OBJECTS:
        if o.name in poses:
            loc = "in the basket" if S.in_basket(poses[o.name]) else "on the table"
            parts.append(f"{o.name} ({o.label}, {loc})")
    return "; ".join(parts)


class Planner:
    def __init__(self, mode: str, vlm: VLMClient | None):
        self.mode = mode          # rules | vlm
        self.vlm = vlm
        self.vlm_ok = None
        self.last_error = ""

    def plan(self, text: str, poses: dict, jpeg: bytes | None) -> Plan:
        base = rule_plan(text, poses)
        if self.mode != "vlm" or self.vlm is None:
            return base
        t0 = time.time()
        try:
            raw = self.vlm.ask(text, jpeg, describe_scene(poses))
            self.vlm_ok, self.last_error = True, ""
        except Exception as e:  # server down / timeout / bad json -> fall back
            self.vlm_ok, self.last_error = False, f"{type(e).__name__}: {e}"
            base.notes.append("VLM unavailable, used rule parser")
            base.latency = time.time() - t0
            return base
        plan = Plan(source="vlm", latency=time.time() - t0)
        act = raw.get("action")
        obj = raw.get("object")
        tgt = raw.get("target") if raw.get("target") in ("basket", "table") else "basket"
        plan.reply = str(raw.get("reply") or "")[:300]
        explicit = mentioned_objects(text)
        # grounding guard: if the user named objects explicitly and the small model
        # disagrees, trust the words and tell the user
        if base.steps and base.steps[0].action == "pick_place" and len(base.steps) > 1:
            plan.steps, plan.notes = base.steps, base.notes + ["multi-object request expanded by rules"]
            plan.reply = plan.reply or base.reply
            return plan
        if act == "pick_place" and obj in S.OBJ_BY_NAME:
            if explicit and obj not in explicit:
                plan.notes.append(f"VLM chose {obj} but you named {explicit[0]}; using yours")
                obj = explicit[0]
            if obj in poses and (S.in_basket(poses[obj]) == (tgt == "basket")):
                plan.notes.append(f"{S.OBJ_BY_NAME[obj].label} is already there")
                plan.steps = []
                plan.reply = plan.reply or f"The {S.OBJ_BY_NAME[obj].label} is already there."
            else:
                plan.steps = [Step("pick_place", obj, tgt)]
        elif act == "home":
            plan.steps = [Step("home")]
        elif act == "say":
            if base.steps and base.steps[0].action == "pick_place":      # model chatted, user clearly gave a command
                plan.steps, plan.notes = base.steps, plan.notes + ["VLM replied with text; rules found a command"]
            else:
                plan.steps = [Step("say")]
        else:
            plan.steps, plan.notes = base.steps, plan.notes + ["VLM output unusable; used rules"]
            plan.reply = plan.reply or base.reply
        return plan
