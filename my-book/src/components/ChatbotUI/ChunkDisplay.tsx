/**
 * Chunk display component (T019)
 *
 * Renders matched content chunks with text preview and source location.
 * Shows chapter, page, and section metadata for each chunk.
 * Supports expandable/collapsible format for readability.
 */

import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';
import type { ChunkReference } from '../../types/chatbot';

/**
 * Chunk display props
 */
interface ChunkDisplayProps {
  /** Array of matched chunks to display */
  chunks: ChunkReference[];
  /** Maximum number of chunks to show initially */
  initialLimit?: number;
}

/**
 * Truncate text with ellipsis
 */
function truncateText(text: string, maxLength: number = 200): string {
  if (!text || text.length <= maxLength) return text || '';
  return text.substring(0, maxLength) + '...';
}

/**
 * Slugify text for URL anchor
 */
function slugify(text: string): string {
  return text
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/(^-|-$)/g, '');
}

/**
 * Single chunk item component
 */
function ChunkItem({ chunk }: { chunk: ChunkReference }): JSX.Element {
  const [expanded, setExpanded] = useState(false);
  const history = useHistory();

  // Build location string
  const locationParts: string[] = [];
  if (chunk.chapter) locationParts.push(chunk.chapter);
  if (chunk.section) locationParts.push(chunk.section);
  if (chunk.page) locationParts.push(`Page ${chunk.page}`);
  const location = locationParts.join(' • ') || 'Source';

  // Determine if text should be truncated
  const shouldTruncate = chunk.text && chunk.text.length > 200;
  const displayText = expanded ? chunk.text : truncateText(chunk.text);

  /**
   * Navigate to chunk location in book
   */
  const handleViewInBook = () => {
    if (chunk.chunk_id === 'selected_text') {
      return;
    }

    if (chunk.chapter) {
      let path = `/docs/${slugify(chunk.chapter)}`;
      if (chunk.section) {
        path += `#${slugify(chunk.section)}`;
      }
      history.push(path);
    }
  };

  return (
    <div className={styles.chunkItem}>
      <div className={styles.chunkText}>
        "{displayText}"
        {shouldTruncate && (
          <button
            className={styles.expandButton}
            onClick={() => setExpanded(!expanded)}
            type="button"
          >
            {expanded ? 'Show less' : 'Show more'}
          </button>
        )}
      </div>
      <div className={styles.chunkMeta}>
        <span>{location}</span>
        {chunk.chunk_id !== 'selected_text' && chunk.chapter && (
          <button
            className={styles.viewInBookButton}
            onClick={handleViewInBook}
            type="button"
          >
            View in book →
          </button>
        )}
      </div>
    </div>
  );
}

/**
 * Chunk display component
 *
 * Shows matched content chunks with metadata.
 * Limits display to prevent overwhelming the user.
 */
export function ChunkDisplay({ chunks, initialLimit = 3 }: ChunkDisplayProps): JSX.Element {
  const [showAll, setShowAll] = useState(false);

  if (!chunks || chunks.length === 0) {
    return null;
  }

  // Filter out selected_text chunks for display (they're shown differently)
  const displayChunks = chunks.filter((c) => c.chunk_id !== 'selected_text');

  if (displayChunks.length === 0) {
    return null;
  }

  const visibleChunks = showAll ? displayChunks : displayChunks.slice(0, initialLimit);
  const hasMore = displayChunks.length > initialLimit;

  return (
    <div className={styles.chunksContainer}>
      <div className={styles.chunksTitle}>Matched Content:</div>
      {visibleChunks.map((chunk, index) => (
        <ChunkItem key={chunk.chunk_id || index} chunk={chunk} />
      ))}
      {hasMore && !showAll && (
        <button
          className={styles.showMoreButton}
          onClick={() => setShowAll(true)}
          type="button"
        >
          Show {displayChunks.length - initialLimit} more chunks
        </button>
      )}
      {showAll && hasMore && (
        <button
          className={styles.showMoreButton}
          onClick={() => setShowAll(false)}
          type="button"
        >
          Show fewer chunks
        </button>
      )}
    </div>
  );
}
