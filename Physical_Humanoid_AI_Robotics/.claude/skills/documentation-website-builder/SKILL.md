---
name: "documentation-website-builder"
description: "Build clean, structured documentation websites: generate page layouts, organize navigation, define section hierarchy, and create multi-page documentation ready for deployment. Use when user needs a full documentation site structure."
version: "1.0.0"
---

# Documentation Website Builder Skill

## When to Use This Skill

- User wants a **documentation website**, docs layout, or multi-page structure  
- User says “create documentation pages,” “build docs site,” or “set up navigation”  
- User needs organized structure for APIs, tutorials, product guides, or technical docs  
- User is preparing documentation for a project, tool, or software library

---

## How This Skill Works

1. **Define documentation purpose** – Identify product, target users, and scope  
2. **Generate site structure** – Create sections, pages, and nested navigation  
3. **Design page templates** – Provide layout for API docs, guides, FAQs, tutorials, etc.  
4. **Build navigation schema** – Sidebar, header, and content hierarchy  
5. **Provide deployment-ready structure** – Export sections in markdown-ready format

---

## Output Format

Provide:

- **Project Summary**: 1–2 sentence overview of what the docs website will cover  
- **Audience Profile**: Who the documentation is designed for  
- **Site Structure**: Full multi-page layout with sections and subsections  
- **Page Templates**: Prebuilt content templates for each type of page  
- **Navigation Schema**: Sidebar menu + page hierarchy  
- **Optional Notes**: Recommendations for styling, frameworks, or deployment (Docsify, Docusaurus, GitBook, MkDocs, etc.)

---

## Example

**Input:**  
"Create a documentation website structure for a machine learning library."

**Output:**

### Project Summary  
A documentation website for a beginner-friendly machine learning library, covering setup, APIs, tutorials, and best practices.

### Audience Profile  
Developers, students, and data scientists learning the library.

### Site Structure  
1. **Introduction**  
   - What is the library?  
   - Key features  
   - Core principles  

2. **Getting Started**  
   - Installation  
   - Basic usage  
   - First ML model  

3. **API Reference**  
   - Data loaders  
   - Model classes  
   - Training utilities  
   - Evaluation functions  

4. **Guides & Tutorials**  
   - Building your first classifier  
   - Working with datasets  
   - Custom model creation  

5. **Advanced Topics**  
   - Hyperparameter tuning  
   - Optimization techniques  
   - Deployment guide  

6. **FAQ & Troubleshooting**  
   - Common errors  
   - Performance tips  

7. **Release Notes**  
   - Version history  
   - Changelog  

### Page Templates  
- **Intro Page Template**: Overview, key points, quick start code  
- **API Page Template**: Function → parameters → examples → notes  
- **Tutorial Template**: Steps → code → explanation → summary  
- **FAQ Template**: Question → short answer → example  

### Navigation Schema  
- Sidebar with collapsible sections matching site structure  
- Subpages nested under major categories  

### Optional Notes  
- Recommended frameworks: **Docusaurus**, **GitBook**, or **MkDocs**  
- Use consistent code formatting and highlight blocks  
- Include search functionality for large docs  
