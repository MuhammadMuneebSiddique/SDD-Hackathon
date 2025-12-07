---
id: 0001
title: Physical AI Book Spec
stage: spec
date: 2025-12-07
surface: project
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book
branch: 001-physical-ai-book
user: ASNA
command: /sp.specify
labels: [specification, docusaurus, robotics, education]
links:
  spec: specs/001-physical-ai-book/spec.md
  ticket:
  adr:
  pr:
files:
  - path: specs/001-physical-ai-book/spec.md
    action: created
    summary: Complete specification for Physical AI & Humanoid Robotics book
  - path: specs/001-physical-ai-book/checklists/requirements.md
    action: created
    summary: Quality checklist for specification validation
tests:
  - type: validation
    result: passed
    summary: Specification quality checklist completed successfully
---

## Prompt

)
🎯 Project Goal

Create a complete Docusaurus book that teaches a 13-week capstone course on Physical AI & Humanoid Robotics, covering ROS 2 → Simulation → Isaac → VLA → Autonomous Humanoid.

Content is created through Claude + SpecKit Plus, using Sub-Agents and Skills with strict task separation.

1) 📘 Book Scope (What the Book Contains)
Core Modules (4-Module Structure — MUST BE INCLUDED)
Module 1 — ROS 2: The Robotic Nervous System

Nodes / Topics / Services

ROS 2 Python (rclpy) bridge for AI agents

URDF basics for humanoids

Module 2 — Digital Twin (Gazebo + Unity)

Physics simulation, collisions, gravity

Environment building + sensors (LiDAR, Depth, IMU)

Unity for HRI visualization

Module 3 — NVIDIA Isaac: The AI-Robot Brain

Isaac Sim photorealistic simulation

Isaac ROS (VSLAM, perception)

Nav2 for humanoid movement

Sim-to-Real workflow

Module 4 — VLA: Vision-Language-Action

Whisper voice input → command

LLM reasoning → ROS 2 actions

Natural language task planning

🎓 Capstone Project

Robot must:

Take voice command →

Plan navigation →

Identify object →

Manipulate object.

📅 13-Week Breakdown

Weeks 1–2 → Physical AI introduction

Weeks 3–5 → ROS 2

Weeks 6–7 → Gazebo simulation

Weeks 8–10 → NVIDIA Isaac

Weeks 11–12 → Humanoid locomotion + grasping

Week 13 → Conversational robotics (GPT + speech + vision)

🖥 Lab Hardware Requirements

Digital Twin Workstation: RTX 4070 Ti+ (VRAM), 64GB RAM, Ubuntu 22.04

Physical AI Edge Kit: Jetson Orin Nano (8GB), RealSense D435i, ReSpeaker Mic

Robot Options:

Budget → Unitree Go2

Miniature humanoid → OP3 / Unitree G1 Mini

Premium → Unitree G1 Humanoid

Cloud Option: AWS GPU for training → deploy to Jetson

2) 🤖 Agent Architecture (Required by Project)
Main Agent (Master Agent)

Routes all tasks

Controls workflow

Approves final content

Ensures sub-agents stay in their own domain

🔹 Sub-Agents (5) — Strict Domain Separation
1. Research & Knowledge Expert

Owns:

Technical accuracy

Module structures

Robotics concepts
Outputs:

Lesson outlines

Correct definitions

Capstone feasibility guidance

2. Technical Documentation Writer

Owns:

Convert outlines → full Docusaurus markdown

Write chapters, labs, hardware guides
Outputs:

Final written content

Code blocks & walkthroughs

3. Quality Validator

Owns:

Accuracy, consistency, reproducibility
Checks:

ROS/Isaac terminology

Lab steps correctness

Cross-links working
Outputs:

Pass/Fail report

4. SEO & Web Optimization Agent

Owns:

Metadata

Page titles & descriptions

Internal linking for navigation

5. Publishing & Versioning Manager

Owns:

Release preparation

Version tagging

Final site-ready bundle

3) 🛠 Skills (Used Across Agents)
Skill 1 — Content Architect

Designs book hierarchy

Places each doc in correct module/week

Skill 2 — Style & Consistency Editor

Ensures tone, heading structure, code formatting

Standardizes layout for all pages

Skill 3 — Documentation Website Builder

Integrates docs into Docusaurus (frontend folder already exists)

Ensures clean build, working navigation

4) 🔄 End-to-End Workflow (Short Version)

Master Agent sets scope for module/week.

Research Expert creates accurate outline + key concepts.

Technical Writer creates markdown pages.

Style Editor enforces consistent formatting.

Quality Validator checks correctness + links + reproducibility.

SEO Agent adds metadata + improves structure.

Publishing Manager versions + publishes the final release.

## Response snapshot

Successfully created specification for the Physical AI & Humanoid Robotics book project. The specification includes:

- 5 prioritized user stories covering core functionality (course access, RAG chatbot, personalization, multilingual support, hardware guides)
- 13 functional requirements defining the system capabilities
- 4 key entities representing the main data concepts
- 8 measurable success criteria with specific metrics

A quality checklist was also created and validated, confirming the specification meets all requirements for proceeding to the planning phase.

## Outcome

- ✅ Impact: Complete specification created for Docusaurus-based robotics education book with RAG chatbot, personalization, and multilingual support
- 🧪 Tests: Specification quality validation passed with all checklist items completed
- 📁 Files: Created spec.md and requirements checklist in feature directory
- 🔁 Next prompts: Ready for planning phase with `/sp.plan`
- 🧠 Reflection: Specification successfully captures all requirements from user input with clear user scenarios and measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: None - specification creation completed successfully
- Graders run and results (PASS/FAIL): PASS - all validation checks passed
- Prompt variant (if applicable): Standard specification creation prompt
- Next experiment (smallest change to try): Proceed to planning phase with `/sp.plan`
