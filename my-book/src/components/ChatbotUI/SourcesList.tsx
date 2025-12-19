/**
 * Sources list component (T018)
 *
 * Renders clickable source references with chapter/section links.
 * Uses @docusaurus/router for navigation to book sections.
 */

import React from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';
import type { SourceReference } from '../../types/chatbot';

/**
 * Sources list props
 */
interface SourcesListProps {
  /** Array of source references to display */
  sources: SourceReference[];
}

/**
 * Sources list component
 *
 * Displays numbered list of sources with clickable links.
 * Handles "selected_text" source specially.
 */
export function SourcesList({ sources }: SourcesListProps): JSX.Element {
  const history = useHistory();

  if (!sources || sources.length === 0) {
    return null;
  }

  /**
   * Navigate to source location in book
   */
  const handleSourceClick = (source: SourceReference) => {
    // Selected text has no navigation
    if (source.chunkId === 'selected_text') {
      return;
    }

    // Construct path from source reference
    if (source.path) {
      let targetPath = source.path;
      if (source.anchor) {
        targetPath += `#${source.anchor}`;
      }
      history.push(targetPath);
    }
  };

  // Deduplicate sources by chunkId
  const uniqueSources = sources.filter(
    (source, index, self) =>
      index === self.findIndex((s) => s.chunkId === source.chunkId)
  );

  return (
    <div className={styles.sourcesList}>
      <div className={styles.sourcesTitle}>Sources:</div>
      <ol style={{ margin: 0, paddingLeft: '20px' }}>
        {uniqueSources.map((source, index) => (
          <li key={source.chunkId || index}>
            {source.chunkId === 'selected_text' ? (
              <span className={styles.sourceItemNonClickable}>
                📎 {source.label}
              </span>
            ) : source.path ? (
              <span
                className={styles.sourceItem}
                onClick={() => handleSourceClick(source)}
                onKeyDown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') {
                    handleSourceClick(source);
                  }
                }}
                role="link"
                tabIndex={0}
              >
                {source.label}
              </span>
            ) : (
              <span className={styles.sourceItemNonClickable}>
                {source.label}
              </span>
            )}
          </li>
        ))}
      </ol>
    </div>
  );
}
