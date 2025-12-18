// frontend/src/hooks/useTextSelection.test.ts
import { renderHook, act } from '@testing-library/react-hooks';
import { useTextSelection } from './useTextSelection';

describe('useTextSelection', () => {
  let originalGetSelection: typeof window.getSelection;

  beforeAll(() => {
    // Store original window.getSelection
    originalGetSelection = window.getSelection;
  });

  afterEach(() => {
    // Restore original window.getSelection after each test
    Object.defineProperty(window, 'getSelection', {
      value: originalGetSelection,
      writable: true,
    });
  });

  test('returns empty string initially if no text is selected', () => {
    Object.defineProperty(window, 'getSelection', {
      value: () => ({
        toString: () => '',
      }),
      writable: true,
    });

    const { result } = renderHook(() => useTextSelection());
    expect(result.current).toBe('');
  });

  test('returns selected text when text is selected', () => {
    Object.defineProperty(window, 'getSelection', {
      value: () => ({
        toString: () => '  Hello World  ',
      }),
      writable: true,
    });

    const { result } = renderHook(() => useTextSelection());
    expect(result.current).toBe('Hello World');
  });

  test('updates selected text on selectionchange event', () => {
    let mockSelection = '';
    Object.defineProperty(window, 'getSelection', {
      value: () => ({
        toString: () => mockSelection,
      }),
      writable: true,
    });

    const { result } = renderHook(() => useTextSelection());
    expect(result.current).toBe('');

    act(() => {
      mockSelection = 'First selection';
      document.dispatchEvent(new Event('selectionchange'));
    });
    expect(result.current).toBe('First selection');

    act(() => {
      mockSelection = 'Second selection';
      document.dispatchEvent(new Event('selectionchange'));
    });
    expect(result.current).toBe('Second selection');
  });

  test('handles removal of selection', () => {
    let mockSelection = 'Some text';
    Object.defineProperty(window, 'getSelection', {
      value: () => ({
        toString: () => mockSelection,
      }),
      writable: true,
    });

    const { result } = renderHook(() => useTextSelection());
    expect(result.current).toBe('Some text');

    act(() => {
      mockSelection = '';
      document.dispatchEvent(new Event('selectionchange'));
    });
    expect(result.current).toBe('');
  });
});
