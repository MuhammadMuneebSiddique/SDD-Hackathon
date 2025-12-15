# Data Model: Physical AI & Humanoid Robotics Book

## Status
**Book Content Creation: COMPLETED** | **Interactive Features: PENDING**

## Entities

### Book Content (COMPLETED)
- **Fields**:
  - id (string, unique)
  - title (string)
  - module (enum: ROS2, Simulation, Isaac, VLA, Capstone)
  - week (integer, 1-13)
  - content (markdown/MDX) - *FULLY POPULATED*
  - prerequisites (array of content IDs)
  - objectives (array of learning objectives)
  - type (enum: lesson, lab, assessment, guide)
- **Relationships**: Contains many Sections, Labs, Assessments
- **Validation**: Title required, content required, valid module value, valid week number
- **Status**: All 13-week course content has been created across 4 core modules

### Section (COMPLETED)
- **Fields**:
  - id (string, unique)
  - bookContentId (string, foreign key)
  - title (string)
  - content (markdown/MDX) - *FULLY POPULATED*
  - order (integer)
  - learningObjectives (array of strings)
- **Relationships**: Belongs to Book Content, contains many Code Examples
- **Validation**: Title required, order required, valid bookContentId
- **Status**: All sections have been created and populated

### User Profile (PENDING IMPLEMENTATION)
- **Fields**:
  - id (string, unique)
  - softwareBackground (string)
  - hardwareBackground (string)
  - preferences (JSON object)
  - createdAt (timestamp)
  - updatedAt (timestamp)
- **Relationships**: Has many Progress Records, Assessment Results
- **Validation**: Background fields required for registered users
- **Status**: Model defined, implementation pending

### Progress Record (PENDING IMPLEMENTATION)
- **Fields**:
  - id (string, unique)
  - userId (string, foreign key)
  - contentId (string, foreign key)
  - completed (boolean)
  - completionDate (timestamp)
  - quizScores (array of objects)
- **Relationships**: Belongs to User Profile and Book Content
- **Validation**: Valid userId and contentId required
- **Status**: Model defined, implementation pending

### Lab Activity (COMPLETED)
- **Fields**:
  - id (string, unique)
  - moduleId (string, foreign key to Book Content)
  - title (string)
  - steps (array of objects with step description and expected outcome) - *FULLY POPULATED*
  - requiredEquipment (array of strings)
  - estimatedTime (integer in minutes)
  - prerequisites (array of content IDs)
- **Relationships**: Belongs to Book Content, has many Lab Submissions (future)
- **Validation**: Title and steps required, estimatedTime positive
- **Status**: All lab activities have been created and populated

## State Transitions

### Content Completion
- Initial → In Progress → Completed
- Triggered by user interaction with progress tracking (PENDING IMPLEMENTATION)

### User Registration
- Anonymous → Registered → Profile Complete
- Profile completion requires software and hardware background information (PENDING IMPLEMENTATION)

## Relationships

- Book Content (1) → Sections (many) - *COMPLETED*
- Book Content (1) → Lab Activities (many) - *COMPLETED*
- User Profile (1) → Progress Records (many) - *PENDING*
- User Profile (1) → Assessment Results (many) - *PENDING*
- Book Content (1) → Assessments (many)