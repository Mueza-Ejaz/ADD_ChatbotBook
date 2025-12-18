# Quickstart Guide: Book Content Creation

This guide provides a quick overview for new contributors to start creating content for the "Physical AI & Humanoid Robotics" textbook.

## 1. Prerequisites

Before you begin, ensure you have:

*   **Git**: Installed and configured on your system.
*   **Node.js & npm/yarn**: Required for Docusaurus.
*   **Text Editor**: A Markdown-friendly editor (e.g., VS Code with Markdown extensions).

## 2. Setting Up Your Development Environment

1.  **Clone the Repository**:
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```
2.  **Checkout Feature Branch**:
    ```bash
    git checkout 001-book-content # Or your assigned feature branch
    ```
3.  **Install Docusaurus Dependencies**:
    ```bash
    cd website
    npm install # or yarn install
    ```
4.  **Start Local Docusaurus Server**:
    ```bash
    cd website
    npm run start # or yarn start
    ```
    This will open the textbook in your browser at `http://localhost:3000`. The server will automatically reload upon content changes.

## 3. Content Structure and Location

All book content resides in the `website/docs/` directory.

```
website/docs/
├── module1/
│   ├── chapter1.md
│   ├── chapter2.md
│   └── ...
├── module2/
│   ├── chapter1.md
│   └── ...
└── ...
website/sidebars.js # Docusaurus sidebar configuration
```

*   **Modules**: Each module corresponds to a directory (e.g., `website/docs/module1/`).
*   **Chapters**: Each chapter is a Markdown (`.md`) or MDX (`.mdx`) file within a module directory (e.g., `website/docs/module1/chapter1.md`).
*   **Sidebar Navigation**: The `website/sidebars.js` file controls the navigation structure. Ensure your new chapters are added to the correct section.

## 4. Creating and Editing Content

1.  **Create a New Chapter**:
    *   Navigate to the appropriate module directory (e.g., `website/docs/module1/`).
    *   Create a new Markdown (`.md`) or MDX (`.mdx`) file for your chapter (e.g., `new-chapter-title.md`).
    *   Add Docusaurus front matter at the top of your file:
        ```markdown
        ---
        sidebar_position: 1 # Position in the sidebar (optional)
        title: My New Chapter Title
        ---
        ```
2.  **Follow Chapter Structure**:
    Each chapter MUST follow the defined sections:
    *   Learning Objectives
    *   Theoretical Concepts
    *   Hands-on Examples
    *   Code Snippets (Python, ROS 2, Gazebo)
    *   Exercises & Assessments
    *   Further Reading
3.  **Markdown/MDX Features**:
    *   **Code Blocks**: Use triple backticks with language specifier for syntax highlighting (e.g., ````python`).
    *   **Images**: `![Alt Text](/path/to/image.png)` (place images in `website/static/img` directory, then reference `/img/image.png`).
    *   **Videos**: Use standard Markdown or embed as React components in MDX for more control.
    *   **Interactive Elements**: For interactive quizzes or simulations, define them in MDX or as React components.

## 5. Verifying Your Changes

*   **Local Server**: Always check your changes by running `npm run start` from `website/` and navigating to your new/edited chapter.
*   **Build Test**: Before pushing, run a build from `website/` to catch any Docusaurus-specific errors:
    ```bash
    npm run build
    ```

## 6. Submitting Your Contributions

1.  **Commit Your Changes**:
    ```bash
    git add .
    git commit -m "feat: Add new chapter: <Chapter Title>" # Follow Conventional Commits
    ```
2.  **Push to Your Branch**:
    ```bash
    git push origin <your-branch-name>
    ```
3.  **Create a Pull Request**: Open a PR from your feature branch to `main`. Ensure all checks pass.