# Feature Specification: Robotics UI Theme Enhancement

**Feature Branch**: `004-robotics-ui-theme`
**Created**: 2025-12-18
**Status**: Draft
**Input**: Create a detailed technical specification for "Phase 4: Enhancement and consistency In UI (professional robotics theme)". Project Context: This is Phase 4 of a hackathon to build an AI-native "Physical AI & Humanoid Robotics" textbook. Phase 1: Docusaurus book content ✓ Phase 2: FastAPI backend with /chat endpoint (in progress) Phase 3: ChatKit chatbot UI integration (in specification) This Phase 4: Apply a professional, consistent "robotics" visual theme across the entire Docusaurus site and the integrated chatbot widget. Core Objective: Transform the visual design of the entire textbook website (Docusaurus) and the Phase 3 chatbot to embody a cohesive, modern, professional "robotics" aesthetic. Ensure all UI elements follow the same design system for a polished, integrated user experience. Detailed Requirements: 1. Design Theme & Principles: Theme Name: "Robotics Professional" Core Attributes: Futuristic, clean, technical, accessible, trustworthy. Inspiration: Modern robotics labs, HMI (Human-Machine Interface) dashboards, technical diagrams. 2. Global Design Tokens (CSS Variables): Define a central set of design variables to ensure consistency. Color Palette: Primary: Deep blue (#1a56db) or teal (#0694a2) for actions and key highlights. Secondary: Steel gray (#6b7280) for secondary elements. Background: Light gray (#f9fafb) for page, pure white (#ffffff) for cards. Accent: Robotic orange/red (#dc2626) or green (#10b981) for warnings/success (like sensor status). Text: High contrast (gray-900 for dark, gray-50 for light). Typography: Use a clean, highly readable font stack (e.g., Inter, system-ui). Establish a clear type scale for headings, body, code, and UI labels. Spacing & Layout: Consistent spacing scale (4px base). Define layout max-widths and card border-radius. Icons & Imagery: Use or source a set of consistent, technical icons (e.g., from Lucide React). Establish guidelines for diagrams and technical images used in the book. 3. Docusaurus Site-Wide Styling: Apply the design tokens to override Docusaurus's default theme. Target key components: navigation bar, sidebar, code blocks, buttons, cards, tables, admonitions (notes, tips, warnings). Ensure the theme works in both light and dark mode, adhering to accessibility (WCAG AA) contrast ratios. 4. Chatbot Widget (Phase 3) Styling: Thematically align the ChatKit component with the new site-wide theme. Style the chat container, message bubbles, input area, and buttons to match the robotics color palette and typography. Add subtle thematic elements to the chatbot: e.g., a minimal robot/gear icon, a status indicator light, or a subtle tech-patterned background. 5. Key Components to Redesign: Header/Navigation: Should look like a technical dashboard header. Code Blocks: Style to resemble a code terminal or robotic control panel. Buttons & Controls: Use clear, purposeful styling for primary and secondary actions. Admonitions (Callouts): Style notes, warnings, and tips to look like system alerts or log entries. Chatbot Interface: Must feel like an integrated part of the site, not a third-party widget. 6. Implementation Strategy: Primary Method: Customize via Docusaurus's src/css/custom.css and by modifying theme configuration files (docusaurus.config.js). Component Swizzling: If needed, carefully "swizzle" specific Docusaurus theme components for deeper customization. ChatKit Styling: Use CSS-in-JS (via Emotion/styled-components) or targeted global CSS to style the ChatKit React components, ensuring no style conflicts. 7. Acceptance Criteria (Checklist): [ ] A cohesive color palette and typography system is defined and documented. [ ] The main Docusaurus site (navigation, sidebar, content area) reflects the new robotics theme. [ ] The Phase 3 chatbot widget is visually integrated and matches the site theme. [ ] The theme supports both light and dark modes accessibly. [ ] All UI states (hover, active, disabled) are styled consistently. [ ] The site maintains full functionality and responsiveness. Output: Generate a complete spec.md file with these sections: 1. Overview & Design Vision 2. Design Token Specification (Color, Typography, Spacing) 3. Docusaurus Component Styling Guide 4. Chatbot Widget Styling Guide 5. Implementation Plan & Technical Notes 6. Accessibility & Responsiveness Checklist 7. Acceptance Criteria.

## 1. Overview & Design Vision

This specification outlines the visual transformation of the "Physical AI & Humanoid Robotics" textbook website and its integrated chatbot. The core objective is to apply a professional, consistent "robotics" visual theme across the entire Docusaurus site and the integrated chatbot widget, ensuring all UI elements follow a unified design system for a polished and integrated user experience. The theme, "Robotics Professional," will embody futuristic, clean, technical, accessible, and trustworthy attributes, drawing inspiration from modern robotics labs, HMI dashboards, and technical diagrams.

## 2. Design Token Specification (Color, Typography, Spacing)

### Color Palette

*   **Primary**: Deep blue (`#1a56db`) or teal (`#0694a2`) for actions and key highlights.
*   **Secondary**: Steel gray (`#6b7280`) for secondary elements.
*   **Background**: Light gray (`#f9fafb`) for page, pure white (`#ffffff`) for cards.
*   **Accent**: Robotic orange/red (`#dc2626`) or green (`#10b981`) for warnings/success (like sensor status).
*   **Text**: High contrast (gray-900 for dark, gray-50 for light).

### Typography

*   **Font Stack**: Use a clean, highly readable font stack (e.g., `Inter`, `system-ui`).
*   **Type Scale**: Establish a clear type scale for headings (H1-H6), body text, code blocks, and UI labels, ensuring hierarchy and readability.

### Spacing & Layout

*   **Spacing Scale**: Consistent spacing scale (e.g., 4px base unit) for margins, paddings, and gaps.
*   **Layout Max-Widths**: Define maximum content widths for optimal readability on large screens.
*   **Card Border-Radius**: Consistent border-radius for card-like elements.

### Icons & Imagery

*   **Icons**: Use or source a set of consistent, technical icons (e.g., from Lucide React) to reinforce the robotics theme.
*   **Imagery Guidelines**: Establish guidelines for diagrams and technical images used in the book to ensure visual consistency with the new theme.

## 3. Docusaurus Component Styling Guide

The following Docusaurus components will be styled to reflect the "Robotics Professional" theme, overriding default Docusaurus styles where necessary. The theme must support both light and dark modes, adhering to accessibility (WCAG AA) contrast ratios.

*   **Navigation Bar**: Header/Navigation should look like a technical dashboard header.
*   **Sidebar**: Consistent styling with the navigation bar, clear active/inactive states.
*   **Code Blocks**: Style to resemble a code terminal or robotic control panel.
*   **Buttons & Controls**: Use clear, purposeful styling for primary and secondary actions, with consistent hover, active, and disabled states.
*   **Cards**: Consistent background, border-radius, and shadow effects.
*   **Tables**: Clean, readable table styles.
*   **Admonitions (Notes, Tips, Warnings)**: Style notes, warnings, and tips to look like system alerts or log entries.

## 4. Chatbot Widget Styling Guide

The Phase 3 chatbot widget must be thematically aligned with the new site-wide theme.

*   **Chat Container**: Style to match the overall robotics aesthetic.
*   **Message Bubbles**: Thematic color scheme and shape for user and AI messages.
*   **Input Area**: Consistent with other input fields on the site.
*   **Buttons**: Match the site's button styling.
*   **Subtle Thematic Elements**: Integrate elements like a minimal robot/gear icon, a status indicator light, or a subtle tech-patterned background into the chatbot interface. The chatbot interface must feel like an integrated part of the site, not a third-party widget.

## 5. Implementation Plan & Technical Notes

### Primary Implementation Methods

*   **Docusaurus Customization**: Utilize `frontend/src/css/custom.css` for global styles and override Docusaurus theme variables by modifying `frontend/docusaurus.config.ts`.
*   **Component Swizzling**: Carefully "swizzle" specific Docusaurus theme components (e.g., `Layout`, `Navbar`, `DocSidebar`) only when deeper customization is required and cannot be achieved via CSS overrides or configuration.

### ChatKit Styling

*   **Method**: Use targeted global CSS within `frontend/src/css/custom.css` to style the ChatKit React components. Prioritize using CSS variables defined in the Global Design Tokens.
*   **Conflict Avoidance**: Ensure styling does not conflict with Docusaurus's default styles or other components.

## 6. Accessibility & Responsiveness Checklist

*   **Accessibility (WCAG AA)**:
    *   [ ] All color combinations (text on background, primary/secondary colors) meet WCAG AA contrast ratio standards in both light and dark modes.
    *   [ ] Interactive elements (buttons, links, form fields) are keyboard-navigable and have clear focus indicators.
    *   [ ] Semantic HTML is used to ensure screen reader compatibility.
*   **Responsiveness**:
    *   [ ] The entire Docusaurus site and chatbot UI adapt gracefully to various screen sizes (desktop, tablet, mobile).
    *   [ ] Navigation, content layout, and interactive elements remain fully functional and aesthetically pleasing on smaller screens.

## 7. Acceptance Criteria

*   [ ] A cohesive color palette and typography system is defined and documented within the `spec.md`.
*   [ ] The main Docusaurus site (navigation, sidebar, content area) reflects the new robotics theme.
*   [ ] The Phase 3 chatbot widget is visually integrated and matches the site theme.
*   [ ] The theme supports both light and dark modes accessibly.
*   [ ] All UI states (hover, active, disabled) are styled consistently.
*   [ ] The site maintains full functionality and responsiveness.