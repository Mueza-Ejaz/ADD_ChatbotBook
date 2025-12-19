// frontend/src/hooks/useTextSelection.ts

import { useState, useEffect } from 'react';

/**
 * Custom React hook to capture and return the currently selected text in the DOM.
 * @returns The selected text as a string, or an empty string if no text is selected.
 */
export const useTextSelection = (): string => {
  const [selectedText, setSelectedText] = useState<string>('');

  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      setSelectedText(selection ? selection.toString().trim() : '');
    };

    document.addEventListener('selectionchange', handleSelectionChange);

    // Initial check in case text is already selected when component mounts
    handleSelectionChange();

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  return selectedText;
};
