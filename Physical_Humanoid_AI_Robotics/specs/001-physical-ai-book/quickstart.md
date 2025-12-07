# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Development Setup

1. **Prerequisites**
   - Node.js 18+ and npm
   - Git
   - GitHub access

2. **Clone and Setup**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   cd frontend
   npm install
   ```

3. **Local Development**
   ```bash
   cd frontend
   npm start
   ```
   This will start the Docusaurus development server at http://localhost:3000

4. **Build for Production**
   ```bash
   cd frontend
   npm run build
   ```

## Content Creation Workflow

1. **Module Structure**
   - Content is organized in `frontend/docs/` by modules:
     - `module-1-ros2/` - ROS 2: The Robotic Nervous System
     - `module-2-simulation/` - Digital Twin (Gazebo + Unity)
     - `module-3-isaac/` - NVIDIA Isaac: The AI-Robot Brain
     - `module-4-vla/` - VLA: Vision-Language-Action
     - `capstone/` - Capstone Project

2. **Creating New Content**
   - Add new markdown files to appropriate module directory
   - Follow the standard frontmatter structure:
   ```markdown
   ---
   title: Your Title
   sidebar_position: [position number]
   description: Brief description
   ---
   ```

3. **Adding Interactive Elements**
   - Use Docusaurus components for:
     - Code blocks with syntax highlighting
     - Mermaid diagrams for visualizations
     - Tabs for different approaches
     - Callout blocks for important information

## Agent-Based Content Generation

1. **Research Phase**
   - Use Research & Knowledge Expert agent to create accurate outlines
   - Ensure technical correctness for ROS 2, Gazebo, Isaac, VLA concepts

2. **Writing Phase**
   - Use Technical Documentation Writer agent to convert outlines to markdown
   - Include code examples, labs, and hardware guides

3. **Validation Phase**
   - Use Quality Validator agent to check correctness and reproducibility
   - Verify all labs are step-by-step reproducible

## Deployment

1. **GitHub Pages Deployment**
   - The site is configured for GitHub Pages deployment
   - Changes to main branch trigger automatic deployment

2. **Versioning**
   - Use Docusaurus versioning for course updates
   - Maintain backward compatibility for existing students