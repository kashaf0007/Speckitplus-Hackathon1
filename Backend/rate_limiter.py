"""
Rate Limiter for Gemini API requests.

Implements a token bucket algorithm with request queuing to prevent
hitting Gemini API rate limits (20 requests/day for free tier).
"""

import asyncio
import time
import logging
from typing import Optional, Callable, Any
from dataclasses import dataclass, field
from collections import deque

logger = logging.getLogger(__name__)


@dataclass
class RateLimitConfig:
    """Configuration for rate limiting."""
    # Gemini free tier limits for gemini-1.5-flash:
    # - 15 RPM (requests per minute)
    # - 1 million TPM (tokens per minute)
    # - 1500 RPD (requests per day)
    requests_per_minute: int = 15  # Conservative limit
    requests_per_day: int = 1500  # Free tier daily limit for gemini-1.5-flash
    min_request_interval: float = 1.0  # Reduced to 1 second for better UX (15 RPM allows this)
    max_queue_size: int = 10  # Max pending requests in queue
    queue_timeout: float = 30.0  # Reduced timeout for faster feedback


class RateLimiter:
    """
    Token bucket rate limiter with request queuing.

    Prevents hitting Gemini API rate limits by:
    1. Enforcing minimum interval between requests
    2. Tracking daily request count
    3. Queuing requests when rate limited
    """

    def __init__(self, config: Optional[RateLimitConfig] = None):
        self.config = config or RateLimitConfig()
        self._last_request_time: float = 0
        self._request_count_today: int = 0
        self._day_start: float = time.time()
        self._lock = asyncio.Lock()
        self._queue: deque = deque()
        self._is_rate_limited: bool = False
        self._rate_limit_reset_time: Optional[float] = None

        logger.info(
            f"RateLimiter initialized: "
            f"min_interval={self.config.min_request_interval}s, "
            f"max_rpm={self.config.requests_per_minute}, "
            f"daily_limit={self.config.requests_per_day}"
        )

    def _reset_daily_count_if_needed(self):
        """Reset daily count if a new day has started."""
        current_time = time.time()
        # Reset if more than 24 hours since day_start
        if current_time - self._day_start >= 86400:  # 24 hours
            self._request_count_today = 0
            self._day_start = current_time
            self._is_rate_limited = False
            self._rate_limit_reset_time = None
            logger.info("Daily request count reset")

    def get_status(self) -> dict:
        """Get current rate limiter status."""
        self._reset_daily_count_if_needed()

        remaining_today = max(0, self.config.requests_per_day - self._request_count_today)
        time_since_last = time.time() - self._last_request_time if self._last_request_time else None

        return {
            "requests_today": self._request_count_today,
            "remaining_today": remaining_today,
            "daily_limit": self.config.requests_per_day,
            "is_rate_limited": self._is_rate_limited,
            "time_since_last_request": time_since_last,
            "min_interval": self.config.min_request_interval,
            "queue_size": len(self._queue),
        }

    def mark_rate_limited(self, retry_after_seconds: float = 15.0):
        """Mark as rate limited (called when 429 is received)."""
        self._is_rate_limited = True
        self._rate_limit_reset_time = time.time() + retry_after_seconds
        logger.warning(f"Rate limited! Will reset in {retry_after_seconds}s")

    def reset_rate_limit(self):
        """Manually reset rate limit state (useful after successful request)."""
        self._is_rate_limited = False
        self._rate_limit_reset_time = None
        logger.info("Rate limit state reset")

    def _can_make_request(self) -> tuple[bool, Optional[str], Optional[float]]:
        """
        Check if a request can be made now.

        Returns:
            Tuple of (can_proceed, error_message, wait_time)
        """
        self._reset_daily_count_if_needed()
        current_time = time.time()

        # Check if still rate limited from previous 429
        if self._is_rate_limited and self._rate_limit_reset_time:
            if current_time < self._rate_limit_reset_time:
                wait_time = self._rate_limit_reset_time - current_time
                return False, "API rate limited. Please wait.", wait_time
            else:
                # Rate limit period has passed
                self._is_rate_limited = False
                self._rate_limit_reset_time = None

        # Check daily limit
        if self._request_count_today >= self.config.requests_per_day:
            time_until_reset = 86400 - (current_time - self._day_start)
            hours = int(time_until_reset // 3600)
            minutes = int((time_until_reset % 3600) // 60)
            return False, f"Daily API limit reached. Resets in {hours}h {minutes}m.", time_until_reset

        # Check minimum interval
        if self._last_request_time:
            time_since_last = current_time - self._last_request_time
            if time_since_last < self.config.min_request_interval:
                wait_time = self.config.min_request_interval - time_since_last
                return False, None, wait_time  # No error, just wait

        return True, None, None

    async def acquire(self) -> tuple[bool, Optional[str]]:
        """
        Acquire permission to make a request.

        Waits if necessary (within timeout), returns False if rate limited.

        Returns:
            Tuple of (success, error_message)
        """
        start_wait = time.time()

        async with self._lock:
            while True:
                can_proceed, error_msg, wait_time = self._can_make_request()

                if can_proceed:
                    # Update tracking
                    self._last_request_time = time.time()
                    self._request_count_today += 1
                    logger.debug(
                        f"Request permitted: {self._request_count_today}/{self.config.requests_per_day} today"
                    )
                    return True, None

                if error_msg:
                    # Hard rate limit (daily or 429)
                    return False, error_msg

                # Need to wait for minimum interval
                if wait_time and wait_time > 0:
                    # Check if we've been waiting too long
                    total_wait = time.time() - start_wait
                    if total_wait + wait_time > self.config.queue_timeout:
                        return False, "Request timed out in queue. Please try again."

                    logger.debug(f"Waiting {wait_time:.1f}s before next request")
                    await asyncio.sleep(wait_time)

    def record_success(self):
        """Record a successful request (for metrics)."""
        pass  # Already tracked in acquire()

    def record_failure(self, is_rate_limit: bool = False, retry_after: float = 60.0):
        """Record a failed request."""
        if is_rate_limit:
            self.mark_rate_limited(retry_after)


# Global rate limiter instance
_rate_limiter: Optional[RateLimiter] = None


def get_rate_limiter() -> RateLimiter:
    """Get or create the global rate limiter instance."""
    global _rate_limiter
    if _rate_limiter is None:
        _rate_limiter = RateLimiter()
    return _rate_limiter
