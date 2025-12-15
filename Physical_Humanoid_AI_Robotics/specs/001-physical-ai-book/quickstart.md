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

## Content Structure (COMPLETED)

1. **Module Structure**
   - Content is organized in `frontend/docs/` by modules:
     - `module-1-ros2/` - ROS 2: The Robotic Nervous System
     - `module-2-simulation/` - Digital Twin (Gazebo + Unity)
     - `module-3-isaac/` - NVIDIA Isaac: The AI-Robot Brain
     - `module-4-vla/` - VLA: Vision-Language-Action
     - `capstone/` - Capstone Project
     - `resources/` - Hardware guides and lab instructions

2. **Content Organization**
   - All 13-week course content has been created across the 4 core modules
   - Each module includes theory, practical labs, and hardware guides
   - Capstone project integrates all concepts from the course

## Interactive Features (PENDING)

1. **RAG Chatbot Implementation**
   - Integration of OpenAI APIs with FastAPI backend
   - Qdrant vector database for content-restricted responses
   - Real-time processing without data storage

2. **Personalization Features**
   - User registration with software/hardware background questions
   - Content adjustment based on user profile
   - Better-Auth integration

3. **Translation Features**
   - Urdu translation capability for chapter content
   - Language preference persistence

## Deployment

1. **GitHub Pages Deployment**
   - The site is configured for GitHub Pages deployment
   - Changes to main branch trigger automatic deployment

2. **Versioning**
   - Use Docusaurus versioning for course updates
   - Maintain backward compatibility for existing students