---
title: "LLM Cognitive Planning"
module: 4
chapter: 2
id: "ch2-llm-planning"
sidebar_position: 2
learning_objectives:
  - "Understand how Large Language Models convert text into structured robot plans"
  - "Recognize intent extraction, entity identification, and constraint handling mechanisms"
  - "Apply prompting techniques to guide LLMs toward robot-compatible output formats"
prerequisites:
  - "Module 1: ROS 2 Fundamentals completed"
  - "Chapter 1: Speech Recognition completed"
related_chapters:
  - "chapter1-whisper"
  - "chapter3-ros2-actions"
  - "chapter4-complete-vla"
keywords:
  - "LLM"
  - "intent recognition"
  - "semantic understanding"
  - "planning"
  - "VLA"
  - "prompting"
difficulty: "Beginner"
estimated_reading_time: "16 minutes"
estimated_word_count: 5000
created_at: "2025-12-08"
chunk_count: 10
searchable_terms:
  - "LLM"
  - "large language model"
  - "intent"
  - "entity"
  - "semantic"
  - "planning"
  - "prompt"
  - "structured output"
  - "few-shot learning"
  - "hallucination"
---

# Chapter 2: LLM Cognitive Planning

## What is an LLM?

A **Large Language Model (LLM)** is an AI system trained on vast amounts of text that learns to predict the next word in a sequence.

**Simple analogy**: Imagine reading billions of books. You learn:
- How sentences are structured
- What words typically follow each other
- How to express ideas clearly
- How to answer questions

LLMs do the same with text data!

### Key LLMs for Robotics

- **ChatGPT/GPT-4**: Conversational, excellent language understanding
- **Claude**: Strong reasoning, safety-focused
- **Llama**: Open-source, local deployment possible
- **Gemini**: Multimodal (text + images), strong reasoning

## How LLMs Extract Intent and Entities

Your robot doesn't need to understand English—it needs to extract **what to do** and **what to act on**.

### Intent Extraction

```
User: "Pick up the blue object"

Intent Analysis:
  - Primary intent: PICK_UP
  - Modifiers: "gently" (implied by fragility)
  - Urgency: Normal (no "quickly", "immediately")

Robot understands: Action=PICK_UP with normal force
```

### Entity Extraction

```
User: "Pick up the blue object on the table"

Entities:
  - Object: "blue object" (color: blue, type: unknown)
  - Location: "on the table" (where to find it)
  - Constraint: "gently" (implied, based on fragility)

Robot understands: What=blue_object, Where=table_surface
```

## Prompting: Guiding LLMs Toward Robot Commands

**Prompting** is the art of asking LLMs to produce structured output for robots.

### Basic Prompt

```
User input: "Pick up the blue object"

Naive prompt:
"What should the robot do?"
LLM response: "The robot should pick up a blue object from the table."
Problem: Still English! Robot can't use this.
```

### Structured Prompt (Better)

```
Prompt:
"Extract the robot action from this user command.
Output as JSON with fields: action, object, location, constraints.
User said: Pick up the blue object"

LLM response:
{
  "action": "pick_up",
  "object": {
    "color": "blue",
    "type": "unspecified"
  },
  "location": "unspecified",
  "constraints": {
    "force": "gentle"
  }
}
```

✅ **Structured output** that robots can parse!

### Advanced Prompting: Few-Shot Learning

```
Prompt:
"Extract robot action. Output as JSON.

Examples:
  Input: 'Place the red cube on the shelf'
  Output: {"action": "place", "object": "cube", "color": "red", "location": "shelf"}

  Input: 'Move the robot forward 2 meters'
  Output: {"action": "move_forward", "distance": 2.0, "unit": "meters"}

Now process:
  Input: 'Pick up the blue object carefully'
  Output:"

LLM response (from learned pattern):
{
  "action": "pick_up",
  "object": "unknown_type",
  "color": "blue",
  "constraints": {"force": "gentle"}
}
```

✅ **Few examples teach LLM the exact format you want!**

## LLM Capabilities for Robots

### Strong Capabilities

| Task | Example | Success Rate |
|------|---------|---|
| **Intent extraction** | "Pick up" from "Pick up the blue object" | 98%+ |
| **Entity identification** | "Blue object" is the what, "table" is the where | 95%+ |
| **Constraint recognition** | "Gently" implies low force | 90%+ |
| **Multi-step planning** | "Pick up A, move to B, place on C" | 85%+ |
| **Clarification** | "Which blue object?" when ambiguous | 90%+ |

### Limitations & Failures

| Problem | Example | Cause |
|---------|---------|-------|
| **Hallucination** | Inventing details not in original command | LLM generates plausible-sounding but false info |
| **Out-of-domain** | "Make me a sandwich" (robot has no gripper) | LLM doesn't know robot capabilities |
| **Ambiguity** | "It" without antecedent | Requires context |
| **Contradiction** | "Gently pick it up forcefully" | Conflicting instructions |
| **Nonsensical** | "Move the table to itself" | Logically impossible |

## Real-World Scenario: Kitchen Robot

Let's trace LLM understanding in a kitchen context:

### User Command
```
"Robot, pour me a glass of water"
```

### LLM Processing
```
Analysis:
  - Action: POUR
  - Object being poured: WATER
  - Recipient: USER
  - Container: GLASS
  - Context: KITCHEN

Constraints inferred from context:
  - Temperature: ROOM_TEMPERATURE (unless specified)
  - Quantity: ~250ml (typical glass)
  - Speed: SLOW (careful pouring)
```

### Structured Output
```json
{
  "action": "pour",
  "object": "water",
  "recipient": "user",
  "container": "glass",
  "quantity_ml": 250,
  "speed": "slow",
  "robot_capability_required": "pouring_mechanism"
}
```

### Robot Response
```
Check capabilities:
  ✅ Find water (refrigerator model added to robot's knowledge)
  ✅ Locate glass (in kitchen cabinets)
  ✅ Pour (requires liquid handling gripper - not available)
  ❌ FAIL: Robot doesn't have pouring mechanism

Robot to user: "I don't have a pouring mechanism. Would you like me to bring you the water bottle instead?"
```

## Comparison: LLM vs. Traditional Programming

### Traditional Robot Programming

```python
def pick_up_blue_object():
    # Find blue objects in scene
    blue_objects = vision.find_by_color('blue')
    if not blue_objects:
        print("No blue objects found")
        return False

    # Pick the closest one
    target = min(blue_objects, key=lambda obj: distance_to(obj))

    # Plan gripper motion
    trajectory = motion_planner.plan_grasp(target)

    # Execute
    robot.execute_trajectory(trajectory)
    return True
```

**Problem**: Code must be written for every command!

### LLM-Based Robot Control

```python
user_command = "Pick up the blue object"

# Single LLM call handles variety:
action_plan = llm.extract_action(user_command)
# Returns: {"action": "pick_up", "object": {"color": "blue"}}

# Same code executes ANY action:
execute_plan(action_plan)
```

**Advantage**: One LLM call handles endless variations!

## LLM Reasoning: Understanding "Why"

LLMs can reason about context:

```
User: "The bowl is on the table. Pick it up."

LLM reasoning:
  - "The bowl" refers to bowl mentioned in previous sentence
  - "it" is pronoun for "bowl"
  - "on the table" provides location
  - "Pick it up" is action on the bowl

Output:
{
  "action": "pick_up",
  "object": "bowl",
  "location": "table"
}
```

vs. Simple rule-based system:
```
Regex: "pick it up"
Action: "pick_up"
Object: "it" (Unknown! Pronoun resolution failed)
```

## Integration with Module 1: ROS 2 Coordination

The LLM output becomes a [ROS 2 message](/module1/ch1-ros2-core) that coordinates the robot system:

**Flow**:
1. Whisper (Chapter 1) publishes text to `/speech/transcribed` topic
2. [ROS 2 Python Agent](/module1/ch2-agent-bridge) receives transcription
3. Agent calls LLM planning service
4. LLM outputs structured JSON action
5. Agent publishes result to `/planning/action` topic
6. [Action Server](/module1/ch1-ros2-core) receives and executes (Chapter 3)

See [Module 1: Python Agents](/module1/ch2-agent-bridge) for detailed coordination patterns and [Chapter 3: ROS 2 Actions](/module4/ch3-ros2-actions) for execution details.

---

## Integration in Complete VLA Pipeline

Here's where LLM fits in the complete system:

```
Chapter 1 (Whisper): Audio → Text
    "Pick up the blue object"
              ↓
Chapter 2 (LLM): Text → Structured Plan
    {action: pick_up, object: blue_object, constraints: {force: gentle}}
              ↓
Chapter 3 (ROS 2 Actions): Plan → Robot Motion
    Execute trajectory via [Action Servers](/module4/ch3-ros2-actions)
              ↓
Chapter 4 (Feedback Loop): Motion → Perception
    Confirm object grasped via VSLAM/sensors ([Module 3](/module3/intro))
```

**LLM's role**: Translate human language to robot-understandable structured plans

---

## Edge Cases: When LLMs Fail

Understanding LLM limitations is critical for robust robot systems:

### Example 1: The Hallucination Problem

```
User: "Get me the xyz object"

LLM might output:
{
  "action": "pick_up",
  "object": "xyz",
  "color": "red",      // ← Hallucinated! User never said color
  "size": "small"      // ← Also hallucinated!
}

Robot searches for "red small xyz object" → Doesn't exist → Failure
```

**Solution**: Always validate LLM output against available objects in the scene. See [Module 3: Vision Systems](/module3/ch1-isaac-sim-fundamentals) for object detection integration.

### Example 2: Out-of-Domain Commands

```
User: "Make me a sandwich"

LLM outputs:
{
  "action": "make",
  "object": "sandwich",
  "ingredients": ["bread", "meat", "cheese"]
}

Robot reality check:
- ✅ Robot has gripper
- ❌ Robot can't use stove
- ❌ Robot can't assemble sandwich
- ❌ Out of scope for robot capabilities

Robot response: "I can bring you sandwich ingredients, but I can't assemble them."
```

**Solution**: Store robot capability profiles. Validate all LLM outputs against them.

### Example 3: Ambiguous Reference

```
User (while pointing): "Pick it up"

LLM gets only text: "Pick it up"
LLM doesn't know what "it" refers to!

Without context:
{
  "action": "pick_up",
  "object": ???      // ← Undefined!
}
```

**Solution**: Pass visual/spatial context to LLM. Include list of visible objects:

```
LLM Prompt:
"User said: 'Pick it up'
Visible objects: [blue_ball, red_cube, green_box]
Which object should robot pick up?"

LLM (with context): "The user is probably pointing at the nearest object or most salient one."
```

---

## Key Takeaways

✓ **LLMs** learn language patterns from massive datasets (not programmed explicitly)
✓ **Intent extraction** identifies what the user wants (pick, place, move, etc.)
✓ **Entity extraction** identifies what objects and locations are involved
✓ **Prompting** guides LLMs toward structured, robot-compatible output
✓ **Few-shot learning** teaches LLM your exact desired output format via examples
✓ **Capabilities**: Intent, entities, constraints, multi-step planning, reasoning
✓ **Limitations**: Hallucination, out-of-domain commands, ambiguity without context
✓ **Integration**: Works within [ROS 2 architecture](/module1/ch1-ros2-core) as a planning service
✓ **Advantage over hard-coding**: Single LLM handles endless command variations

---

## Next: Chapter 3

Now you have a structured action plan from the LLM. But how does the robot **execute** it in the real world? In **[Chapter 3: ROS 2 Action Integration](/module4/ch3-ros2-actions)**, you'll learn how robots translate plans into actual motion using action servers and trajectory execution.

---

**Learning Outcome**: You now understand how LLMs extract meaning from text, how prompting guides them toward robot commands, why structured output is critical, and why they're more flexible than hard-coded robot programs.
