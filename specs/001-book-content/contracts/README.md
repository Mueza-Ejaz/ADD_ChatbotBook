# Contracts for Book Content Creation

In the context of book content creation, "contracts" primarily refer to the structural and formatting conventions that ensure consistency and proper rendering within Docusaurus.

## Content Structure Contracts

These are defined in `data-model.md` and enforced through `quickstart.md` guidelines:

*   **Module Structure**: Content organized under `docs/moduleX/` directories.
*   **Chapter Structure**: Each chapter is a Markdown/MDX file within a module directory, adhering to a specific internal section layout (Learning Objectives, Theory, Examples, etc.).
*   **Navigation Structure**: Defined by `sidebars.js`, dictating how chapters and modules appear in the Docusaurus navigation.

## Formatting and Media Contracts

*   **Markdown/MDX Syntax**: Adherence to standard Markdown and MDX capabilities for text, lists, tables, etc.
*   **Code Snippets**: Use of fenced code blocks with language identifiers for syntax highlighting.
*   **Image Embedding**: Consistent practices for embedding images, including paths and alt text.
*   **Interactive Elements**: Conventions for embedding or developing interactive components.

These contracts ensure that all content contributions are harmonized and integrate seamlessly into the final textbook.
