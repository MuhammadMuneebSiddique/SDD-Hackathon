# Data Model: Physical AI & Humanoid Robotics Book

## Entities

### Book Content
- **Fields**:
  - id (string, unique)
  - title (string)
  - module (enum: ROS2, Simulation, Isaac, VLA, Capstone)
  - week (integer, 1-13)
  - content (markdown/MDX)
  - prerequisites (array of content IDs)
  - objectives (array of learning objectives)
  - type (enum: lesson, lab, assessment, guide)
- **Relationships**: Contains many Sections, Labs, Assessments
- **Validation**: Title required, content required, valid module value, valid week number

### Section
- **Fields**:
  - id (string, unique)
  - bookContentId (string, foreign key)
  - title (string)
  - content (markdown/MDX)
  - order (integer)
  - learningObjectives (array of strings)
- **Relationships**: Belongs to Book Content, contains many Code Examples
- **Validation**: Title required, order required, valid bookContentId

### User Profile
- **Fields**:
  - id (string, unique)
  - softwareBackground (string)
  - hardwareBackground (string)
  - preferences (JSON object)
  - createdAt (timestamp)
  - updatedAt (timestamp)
- **Relationships**: Has many Progress Records, Assessment Results
- **Validation**: Background fields required for registered users

### Progress Record
- **Fields**:
  - id (string, unique)
  - userId (string, foreign key)
  - contentId (string, foreign key)
  - completed (boolean)
  - completionDate (timestamp)
  - quizScores (array of objects)
- **Relationships**: Belongs to User Profile and Book Content
- **Validation**: Valid userId and contentId required

### Lab Activity
- **Fields**:
  - id (string, unique)
  - moduleId (string, foreign key to Book Content)
  - title (string)
  - steps (array of objects with step description and expected outcome)
  - requiredEquipment (array of strings)
  - estimatedTime (integer in minutes)
  - prerequisites (array of content IDs)
- **Relationships**: Belongs to Book Content, has many Lab Submissions (future)
- **Validation**: Title and steps required, estimatedTime positive

## State Transitions

### Content Completion
- Initial → In Progress → Completed
- Triggered by user interaction with progress tracking

### User Registration
- Anonymous → Registered → Profile Complete
- Profile completion requires software and hardware background information

## Relationships

- Book Content (1) → Sections (many)
- Book Content (1) → Lab Activities (many)
- User Profile (1) → Progress Records (many)
- User Profile (1) → Assessment Results (many)
- Book Content (1) → Assessments (many)