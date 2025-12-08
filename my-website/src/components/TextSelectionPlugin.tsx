/**
 * Text Selection Plugin Component
 *
 * Allows users to select text on the page and send it to the RAG chatbot
 * for context-specific questions. This is a placeholder for full implementation.
 *
 * Features to implement:
 * - Detect text selection on page
 * - Show tooltip with option to query
 * - Send selected text to RAG API as override context
 */

import React, { useEffect, useRef, useState } from 'react';
import styles from './TextSelectionPlugin.module.css';

interface TextSelectionPluginProps {
  onSelection?: (selectedText: string) => void;
}

export const TextSelectionPlugin: React.FC<TextSelectionPluginProps> = ({
  onSelection,
}) => {
  const [selectedText, setSelectedText] = useState('');
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });
  const [showTooltip, setShowTooltip] = useState(false);
  const tooltipRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const handleTextSelection = () => {
      const selection = window.getSelection();

      if (selection && selection.toString().length > 20) {
        // Only show for selections > 20 characters
        setSelectedText(selection.toString());

        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Position tooltip above selection
        setTooltipPosition({
          x: rect.left + rect.width / 2,
          y: rect.top - 50,
        });

        setShowTooltip(true);
      } else {
        setShowTooltip(false);
      }
    };

    const handleClickOutside = () => {
      setShowTooltip(false);
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  const handleAskAbout = () => {
    if (onSelection) {
      onSelection(selectedText);
    }
    setShowTooltip(false);
  };

  if (!showTooltip || !selectedText) {
    return null;
  }

  return (
    <div
      ref={tooltipRef}
      className={styles.tooltip}
      style={{
        top: `${tooltipPosition.y}px`,
        left: `${tooltipPosition.x}px`,
      }}
    >
      <button
        className={styles.button}
        onClick={handleAskAbout}
        title="Ask the RAG chatbot about this text"
      >
        🤖 Ask about this
      </button>
    </div>
  );
};

export default TextSelectionPlugin;
