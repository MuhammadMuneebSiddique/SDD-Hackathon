# Claude Code & Spec-Kit Requirements

This specification defines all required features and development rules for the **Physical AI & Humanoid Robotics** unified book project using Claude Code, Spec-Kit Plus, and Docusaurus.

---

# 📘 PROJECT REQUIREMENTS

## 1. AI/Spec-Driven Book Creation
- Create a complete book using **Docusaurus**.
- Deploy the book to **GitHub Pages**.
- Use **Spec-Kit Plus** and **Claude Code** to write and manage book content.
- All book content must be generated, structured, and validated through Claude workflows.

---

## 2. Integrated RAG Chatbot Development
- Build and embed a **RAG (Retrieval-Augmented Generation) chatbot** in the Docusaurus book.
- The chatbot must use:
  - **OpenAI Agents / ChatKit SDKs**
  - **FastAPI**
  - **Neon Serverless Postgres**
  - **Qdrant Cloud (Free Tier)** vector database
- Chatbot must:
  - Answer questions based **strictly** on the book’s content.
  - Support answering questions based on **only the text selected by the user**.

---

## 3. Base Functionality Scoring
- Participants receive **100 points** for completing all core features.

---

## 4. Bonus: Claude Code Sub-Agents & Skills (+50 Points)
- Implement reusable intelligence using:
  - Claude Code Sub-Agents  
  - Claude Code Skills
- Integrate into the book workflow (writing, editing, validation, automation, etc.)

---

## 5. Bonus: Signup/Signin with Better-Auth (+50 Points)
- Use **Better-Auth** for full authentication flow.
- At signup, ask questions about the user’s:
  - Software background  
  - Hardware background  
- Store responses to support **personalized content delivery**.

---

## 6. Bonus: Chapter Personalization (+50 Points)
- Logged-in users can press a button at the start of each chapter to:
  - Personalize chapter content based on their background profile.

---

## 7. Bonus: Urdu Translation Button (+50 Points)
- Each chapter includes a button that:
  - Translates chapter content to **Urdu** instantly.

---

# ⚙️ CLAUDE CODE RULES  
*(As required by Spec-Kit Plus & Claude Code runtime)*

## 🤖 Role & Purpose
You are an expert AI system specializing in **Spec-Driven Development (SDD)**.  
Your job is to work with the architect to build the project using:
- Specs  
- Plans  
- Task files  
- MCP tool execution  
- Code changes with testable diffs  

---

## 🧩 Task Context
**Surface:** Project-level execution  
**Success Criteria:**
- All outputs follow exact user intent  
- Every user prompt generates a **Prompt History Record (PHR)**  
- Suggest ADRs for significant architectural decisions  
- All changes are small, tested, and reference specific code

---

# 📂 Core Guarantees (Product Promise)

### 1. Record Every User Input (PHR)
- Every user message must produce a PHR under:
  - `history/prompts/constitution/`
  - `history/prompts/<feature-name>/`
  - `history/prompts/general/`

### 2. ADR Suggestions
For architecturally significant decisions, respond with:

**📋 Architectural decision detected: <summary>. Document? Run `/sp.adr <title>`**

Never auto-create ADRs.

---

# 🛠 Development Guidelines

## 1. Authoritative Source Mandate
- Always use **MCP servers**, **CLI tools**, and **file operations** to gather info or make changes.
- Do not assume or invent solutions.

## 2. Execution Flow
- Treat MCP servers as primary tools for discovery and execution.
- Prefer CLI interactions over internal reasoning.

## 3. PHR Creation Requirements
Every response must:
- Detect stage  
- Generate title and slug  
- Fill all template fields  
- Write to correct directory  
- Confirm file path and validity  

PHR must include:
- Full user input (verbatim)  
- Key assistant output  
- Stage, metadata, labels, routes  
- Updated using agent file tools only  

---

## 4. Explicit ADR Suggestion Rules
Trigger condition:
- Significant architecture choice  
- Multiple options  
- Long-term consequences  

Ask for user consent to create ADR.

---

## 5. Human-as-a-Tool Strategy
Ask the user when:
1. Requirements are unclear  
2. Missing dependencies  
3. Multiple architecture options exist  
4. Major milestone is completed  

---

# 📋 Execution Contract for Every Request

1. Confirm surface & success criteria  
2. List constraints and non-goals  
3. Produce output with acceptance checks  
4. List risks or follow-ups (max 3)  
5. Create PHR  
6. Suggest ADR if applicable  

---

# ✔ Minimum Acceptance Criteria
- Testable output  
- Explicit error paths  
- Small diff  
- Accurate code references  
- No invented APIs or secrets  

---

# 🏗 Architect Guidelines (Planning Requirements)

1. **Scope & Dependencies**  
2. **Key Decisions & Rationale**  
3. **API Contracts**  
4. **Non-Functional Requirements**  
5. **Data & Migration Strategy**  
6. **Operational Readiness**  
7. **Risk Analysis**  
8. **Validation & Testing**  
9. **ADR Linking**  

---

# 📁 Project Structure

- `.specify/memory/constitution.md`  
- `specs/<feature>/spec.md`  
- `specs/<feature>/plan.md`  
- `specs/<feature>/tasks.md`  
- `history/prompts/`  
- `history/adr/`  
- `.specify/` templates and scripts  

---

# 🧪 Code Standards
Refer to `.specify/memory/constitution.md` for:
- Quality  
- Testing  
- Performance  
- Security  
- Architecture principles  

## Active Technologies
- Markdown/MDX for Docusaurus v3 (Node.js-based) + Docusaurus, Node.js 18+, npm package manager (001-docusaurus-book)
- Static file storage for documentation content (no database needed) (001-docusaurus-book)
- Markdown/MDX for Docusaurus v3 (Node.js-based) + Docusaurus, Node.js 18+, npm package manager + Docusaurus 3.x, React 18+, Node.js 18+, npm, GitHub Pages, OpenAI APIs, FastAPI, Neon Serverless Postgres, Qdrant Cloud (001-physical-ai-book)
- Static file storage for documentation content (no database needed), with optional vector database (Qdrant Cloud) for RAG chatbot (001-physical-ai-book)

## Recent Changes
- 001-docusaurus-book: Added Markdown/MDX for Docusaurus v3 (Node.js-based) + Docusaurus, Node.js 18+, npm package manager
