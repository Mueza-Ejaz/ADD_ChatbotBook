# Data Model: Book Content Creation

This document outlines the logical data model for the book content, primarily reflecting its hierarchical structure within Docusaurus.

## Entities

### Module
*   **Description**: A top-level organizational unit for the textbook content. Each module represents a major thematic area of "Physical AI & Humanoid Robotics".
*   **Fields**:
    *   `id`: Unique identifier (e.g., "module1").
    *   `title`: Human-readable title (e.g., "The Robotic Nervous System (ROS 2)").
    *   `description`: Brief overview of the module's scope and learning goals.
    *   `chapters`: A list of `Chapter` entities belonging to this module.
*   **Relationships**: Contains multiple `Chapter` entities.

### Chapter
*   **Description**: A discrete section of book content within a `Module`, focusing on a specific topic.
*   **Fields**:
    *   `id`: Unique identifier (e.g., "chapter1.1").
    *   `module_id`: Identifier of the parent `Module`.
    *   `title`: Human-readable title (e.g., "Introduction to ROS 2").
    *   `file_path`: Relative path to the Markdown/MDX content file (e.g., `docs/module1/chapter1.md`).
    *   `metadata`: Any Docusaurus-specific front matter (e.g., `sidebar_label`, `slug`).
    *   `sections`: A list of `Content Section` types present in the chapter.
*   **Relationships**: Belongs to one `Module`. Composed of multiple `Content Section` types.

### Content Section (Abstract Type)
*   **Description**: Represents a defined structural element within a `Chapter` that guides content authors. Not a distinct data entity in storage, but a logical component of the chapter's content.
*   **Types**:
    *   `Learning Objectives`
    *   `Theoretical Concepts`
    *   `Hands-on Examples`
    *   `Code Snippets`
    *   `Exercises & Assessments`
    *   `Further Reading`
    *   `Diagrams/Images`
    *   `Videos`
    *   `Interactive Elements`

### Code Snippet
*   **Description**: A block of executable code embedded within a `Content Section` (specifically "Hands-on Examples" or "Code Snippets").
*   **Fields**:
    *   `language`: Programming language (e.g., "python", "ros2", "gazebo").
    *   `content`: The actual code block.
    *   `description`: Explanation of the code's purpose.
    *   `testable`: Boolean indicating if the snippet should be verifiable.

### Interactive Element
*   **Description**: A dynamic component embedded within a `Chapter` for user engagement.
*   **Fields**:
    *   `type`: Type of interactive element (e.g., "quiz", "simulation").
    *   `configuration`: JSON or YAML data to configure the interactive element.
    *   `content`: Associated questions, scenarios, etc.
*   **Relationships**: Embedded within a `Chapter`.

## Relationships

*   One `Module` contains many `Chapters`.
*   One `Chapter` belongs to one `Module`.
*   One `Chapter` is composed of various `Content Section` types, which are represented within its Markdown/MDX file.
*   `Code Snippet` and `Interactive Element` are specific types of content that appear within `Chapters`.

## Validation Rules (Examples)

*   All `Modules` and `Chapters` MUST have a unique `id` and `title`.
*   Every `Chapter` MUST belong to an existing `Module`.
*   `file_path` for `Chapters` MUST point to a valid Markdown/MDX file within the `docs/` structure.
*   `Code Snippets` MUST specify a supported `language` for syntax highlighting.
