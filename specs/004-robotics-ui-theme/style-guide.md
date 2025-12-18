# Robotics UI Theme Style Guide

This document outlines the key styling principles and implementations for the "Robotics Professional" theme used in the Docusaurus textbook. It serves as a reference for future development and ensures consistency across the UI.

## 1. Color Palette

The theme leverages a set of CSS variables defined in `frontend/src/css/custom.css` for both light and dark modes.

### Light Mode

*   `--ifm-color-primary`: `#1a56db` (Deep blue)
*   `--ifm-color-secondary`: `#6b7280` (Steel gray)
*   `--ifm-background-color`: `#f9fafb` (Light gray for page background)
*   `--ifm-background-surface-color`: `#ffffff` (Pure white for cards/surfaces)
*   `--ifm-font-color-base`: `rgb(17 24 39)` (Gray-900)
*   `--ifm-font-color-secondary`: `rgb(107 114 128)` (Gray-500)
*   `--ifm-color-success`: `#10b981`
*   `--ifm-color-danger`: `#dc2626`
*   `--ifm-color-warning`: `#fbbf24`
*   `--ifm-color-info`: `#3b82f6`

### Dark Mode

*   `--ifm-color-primary`: `#3b82f6` (Slightly lighter blue)
*   `--ifm-color-secondary`: `#9ca3af` (Lighter steel gray)
*   `--ifm-background-color`: `#1a202c` (Dark background)
*   `--ifm-background-surface-color`: `#2d3748` (Darker surface color)
*   `--ifm-font-color-base`: `rgb(249 250 251)` (Gray-50)
*   `--ifm-font-color-secondary`: `rgb(156 163 175)` (Gray-400)
*   `--ifm-color-success`: `#34d399`
*   `--ifm-color-danger`: `#ef4444`
*   `--ifm-color-warning`: `#facc15`
*   `--ifm-color-info`: `#60a5fa`

## 2. Typography

The theme uses the 'Inter' font family for all text, with fallbacks to system fonts.

*   `--ifm-font-family-base`: `'Inter', system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif, 'Apple Color Emoji', 'Segoe UI Emoji', 'Segoe UI Symbol'`
*   `--ifm-font-family-monospace`: `SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace`
*   Headings (`h1` to `h6`) use `--ifm-font-family-base` with defined font sizes and a bold weight (`--ifm-heading-font-weight: 700`).
*   Base font size: `--ifm-font-size-base: 1rem`.

## 3. Component Styling

Styles for core Docusaurus components and the custom Chatbot widget are defined in `frontend/src/css/custom.css`.

### Navbar

*   Background: `--ifm-background-surface-color`
*   Brand/Title color: `--ifm-font-color-base`
*   Link hover: `--ifm-color-primary`
*   Active link: `--ifm-color-primary`
*   Dark mode adjustments for background and brand/title colors.

### Sidebar

*   List background: `--ifm-background-color`
*   Link color: `--ifm-font-color-base`
*   Active link: `background-color: var(--ifm-color-primary-lightest); color: var(--ifm-color-primary-darkest);` (light mode) and `background-color: var(--ifm-color-primary-dark); color: white;` (dark mode).

### Markdown Content Elements

*   **Buttons**: The `.button` class provides consistent font, weight, border-radius, and transition. Primary buttons (`.button--primary`) use `--ifm-color-primary` for background and white text, with a darker primary color on hover.
*   **Cards**: The `.card` class sets background to `--ifm-background-surface-color`, border, border-radius, and a subtle box-shadow for depth. Hover states increase the box-shadow. Dark mode adjusts border color and box-shadow.
*   **Tables**: Markdown tables (`.markdown table`) have collapse borders, padding, and text alignment. Headers (`th`) use `--ifm-background-surface-color` and `font-weight: var(--ifm-font-weight-semibold)`. Even rows are striped using `--ifm-background-color`. Dark mode adjusts border and header background colors.
*   **Admonitions**: `.theme-admonition` provides a left border, padding, and border-radius. Specific admonition types (`-note`, `-tip`, `-warning`, `-danger`, `-info`) use theme colors for their border and a transparent colored background.

### Chatbot Widget

The Chatbot widget's external elements are styled via dedicated classes in `custom.css`, while internal elements leverage ChatKit's theming capabilities by mapping to `--ifm` variables in `frontend/src/components/Chatbot/ChatWidget.tsx`.

*   **Main Container (`.chat-window`)**: Uses `max-width` and `max-height` for responsiveness, along with `--ifm-background-color`, `--ifm-background-surface-color`, border-radius, and box-shadow. Includes media queries for smaller screens.
*   **Toggle Button (`.chat-toggle-button`)**: The button now uses SVG icons for open/close states, styled with `--ifm-color-primary` background, white color, and a circular shape.
*   **Header (`.chat-header`)**: Uses `--ifm-color-primary` for background and white text.
*   **Message Bubbles (`.message`, `.message.user`, `.message.ai`)**: Styles for padding, border-radius, and max-width. User messages use `--ifm-color-primary-lightest` background, AI messages use `--ifm-background-surface-color`.
*   **Input Area (`.message-input`)**: Uses flexbox for layout, with a styled input field and send button.

This style guide provides a high-level overview of the theme implementation. For detailed CSS properties, refer to `frontend/src/css/custom.css`.
