# Database Schema Contract

**Feature**: 004-vla-pipeline | **Status**: Phase 2 Design Complete | **Date**: 2025-12-09

**Tech Stack**: Neon Postgres (serverless) + Qdrant (vector store) | **Branch**: `004-vla-pipeline`

---

## Overview

This document defines the complete database schema for the VLA humanoid robotics textbook RAG system. It covers:

1. **Neon Postgres Schema** (relational data, metadata, user accounts)
2. **Qdrant Vector Store Schema** (semantic embeddings for RAG)
3. **Data Relationships & Integrity Constraints**
4. **Indexing Strategy** (query performance)
5. **Migration & Rollback** procedures

---

## Part 1: Neon Postgres Schema

### Connection Details

```bash
# Neon Postgres connection string template
postgresql://user:password@ep-XXXXX.us-east-1.neon.tech/textbook_rag?sslmode=require
```

### Table 1: `chunks` — Textbook Content Chunks

Stores semantically meaningful excerpts from the textbook with embeddings and metadata.

```sql
CREATE TABLE chunks (
  -- Identifiers
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

  -- Source Reference
  chapter_id VARCHAR(64) NOT NULL,  -- e.g., "module3/chapter3-vslam"
  module_number INTEGER NOT NULL CHECK (module_number >= 1 AND module_number <= 4),
  section_name VARCHAR(256),        -- e.g., "Chapter 3: Isaac ROS VSLAM"
  heading VARCHAR(256),              -- e.g., "What is VSLAM?"

  -- Content
  content TEXT NOT NULL,             -- 400–800 tokens of textbook text
  token_count INTEGER NOT NULL,      -- Actual token count (via tiktoken)

  -- Embedding
  embedding vector(1536) NOT NULL,   -- OpenAI text-embedding-3-small

  -- Metadata
  keywords TEXT[] NOT NULL,          -- Array of extracted keywords
  learning_objectives TEXT[],        -- Related learning outcomes
  difficulty_level VARCHAR(32) NOT NULL CHECK (difficulty_level IN ('Beginner', 'Intermediate', 'Advanced')),

  -- URL & Navigation
  url_anchor VARCHAR(512) NOT NULL,  -- e.g., "/docs/module3/chapter3-vslam#what-is-vslam"

  -- Audit Trail
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  version INTEGER DEFAULT 1
);

-- Indexes for fast retrieval
CREATE INDEX idx_chapters_module ON chunks(module_number);
CREATE INDEX idx_chapters_difficulty ON chunks(difficulty_level);
CREATE INDEX idx_chapters_chapter_id ON chunks(chapter_id);
CREATE INDEX idx_chapters_keywords ON chunks USING GIN(keywords);
CREATE INDEX idx_chapters_embedding ON chunks USING ivfflat(embedding vector_cosine_ops) WITH (lists = 100);
```

**Purpose**: Central content store for RAG retrieval
**Record Count**: ~2,500–3,000 chunks across 15 chapters
**Storage**: ~500 MB (text + embeddings)
**Access Pattern**: Vector similarity search (Qdrant mirrors this), metadata filtering

---

### Table 2: `users` — User Accounts & Profiles

Stores user authentication and profile information.

```sql
CREATE TABLE users (
  -- Identifiers
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

  -- Authentication
  email VARCHAR(255) NOT NULL UNIQUE,
  auth_provider VARCHAR(32) NOT NULL CHECK (auth_provider IN ('email', 'github', 'google', 'betterauth')),
  auth_id VARCHAR(512),              -- External auth provider ID (if OAuth)
  password_hash VARCHAR(255),        -- bcrypt hash (if email-based auth)

  -- Profile
  name VARCHAR(255),
  avatar_url VARCHAR(512),

  -- Account Status
  is_active BOOLEAN DEFAULT TRUE,
  email_verified BOOLEAN DEFAULT FALSE,
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  last_login_at TIMESTAMP
);

-- Indexes
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_auth_id ON users(auth_id);
CREATE UNIQUE INDEX idx_users_active_email ON users(email) WHERE is_active = TRUE;
```

**Purpose**: User authentication and account management
**Record Count**: ~1,000–10,000 users (depending on adoption)
**Access Pattern**: Email/auth_id lookups for login, profile updates

---

### Table 3: `user_preferences` — Personalization Settings

Stores user learning preferences and UI customization.

```sql
CREATE TABLE user_preferences (
  -- Identifiers
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,

  -- Learning Preferences
  difficulty_level VARCHAR(32) DEFAULT 'Beginner' CHECK (difficulty_level IN ('Beginner', 'Intermediate', 'Advanced')),
  preferred_language VARCHAR(5) DEFAULT 'en' CHECK (preferred_language IN ('en', 'es', 'fr', 'zh', 'ja')),

  -- UI Preferences
  ui_theme VARCHAR(16) DEFAULT 'auto' CHECK (ui_theme IN ('light', 'dark', 'auto')),

  -- Content Preferences
  bookmarked_chapters TEXT[],        -- Array of chapter IDs (e.g., ["module3/chapter3-vslam"])
  learning_path_preference VARCHAR(32) DEFAULT 'sequential', -- 'sequential' or 'custom'

  -- Audit
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Indexes
CREATE INDEX idx_user_prefs_user_id ON user_preferences(user_id);
```

**Purpose**: Personalize RAG responses based on user proficiency and preferences
**Record Count**: One per user (~1,000–10,000)
**Access Pattern**: User ID lookups when building context for RAG queries

---

### Table 4: `chat_history` — Conversation Logs

Stores all user queries, responses, and feedback for improvement.

```sql
CREATE TABLE chat_history (
  -- Identifiers
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

  -- Query & Response
  user_query TEXT NOT NULL,
  llm_response TEXT NOT NULL,

  -- Retrieved Chunks (for context)
  retrieved_chunk_ids UUID[] NOT NULL,  -- IDs from chunks table
  chunk_count INTEGER NOT NULL,         -- Number of chunks retrieved

  -- Response Quality Metrics
  response_tokens INTEGER,              -- Tokens in response
  query_tokens INTEGER,                 -- Tokens in query
  retrieval_time_ms INTEGER,            -- Vector DB retrieval time
  total_time_ms INTEGER,                -- End-to-end latency

  -- User Feedback
  feedback_useful BOOLEAN,              -- "Was this response helpful?"
  feedback_citations_correct BOOLEAN,   -- "Were the citations accurate?"
  feedback_text TEXT,                   -- Optional detailed feedback

  -- Audit
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Indexes
CREATE INDEX idx_chat_user_id ON chat_history(user_id);
CREATE INDEX idx_chat_created ON chat_history(created_at);
CREATE INDEX idx_chat_feedback ON chat_history(feedback_useful) WHERE feedback_useful IS NOT NULL;
```

**Purpose**: Audit trail, user behavior analysis, model training data
**Record Count**: ~10,000–100,000 interactions (depends on user base)
**Access Pattern**: User ID lookups (user's chat history), time-range queries (analytics)

---

### Table 5: `feedback` — Detailed Feedback & Quality Issues

Stores structured feedback for model improvement and content refinement.

```sql
CREATE TABLE feedback (
  -- Identifiers
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chat_id UUID NOT NULL REFERENCES chat_history(id) ON DELETE CASCADE,
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,

  -- Feedback Type
  feedback_type VARCHAR(32) NOT NULL CHECK (feedback_type IN (
    'response_accuracy',
    'citation_accuracy',
    'relevance',
    'missing_context',
    'duplicate_info',
    'harmful_response',
    'other'
  )),

  -- Feedback Content
  rating INTEGER CHECK (rating >= 1 AND rating <= 5),  -- 1-5 scale
  comment TEXT,

  -- What went wrong (if applicable)
  issue_category VARCHAR(64),
  issue_description TEXT,

  -- Resolution
  resolved BOOLEAN DEFAULT FALSE,
  resolved_by_admin VARCHAR(255),
  resolution_notes TEXT,

  -- Audit
  created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
  resolved_at TIMESTAMP
);

-- Indexes
CREATE INDEX idx_feedback_user_id ON feedback(user_id);
CREATE INDEX idx_feedback_type ON feedback(feedback_type);
CREATE INDEX idx_feedback_resolved ON feedback(resolved);
```

**Purpose**: Capture structured quality feedback, identify model failures, guide content updates
**Record Count**: ~5–20% of chat interactions (depends on engagement)
**Access Pattern**: Type lookups (e.g., all "citation_accuracy" issues), unresolved issues list

---

## Part 2: Data Relationships

### Entity Relationship Diagram (Conceptual)

```
┌─────────────────────┐
│      users          │
│  ├─ id (PK)         │
│  ├─ email           │
│  └─ auth_provider   │
└──────────┬──────────┘
           │
       1:1 │
           │
    ┌──────┴─────────────────────┐
    │                             │
    v                             v
┌─────────────────────────┐  ┌──────────────────────┐
│  user_preferences       │  │  chat_history (N)    │
│  ├─ user_id (FK)        │  │  ├─ id (PK)          │
│  ├─ difficulty_level    │  │  ├─ user_id (FK)     │
│  └─ bookmarked_chapters │  │  ├─ user_query       │
└─────────────────────────┘  │  ├─ llm_response     │
                              │  └─ retrieved_chunk_ │
                              │      ids (FK array)  │
                              └──────────┬───────────┘
                                         │
                                      1:N │
                                         v
                           ┌──────────────────────┐
                           │    chunks (N)        │
                           │  ├─ id (PK)          │
                           │  ├─ chapter_id       │
                           │  ├─ content          │
                           │  └─ embedding        │
                           └──────────────────────┘
                                         ^
                                      N:M │
                                         │
                              ┌──────────┴───────────┐
                              │                      │
                              v                      v
                    ┌──────────────────────┐  ┌──────────────┐
                    │   chat_history       │  │  feedback    │
                    │  (retrieved_chunk_ids)  ├─ id          │
                    └──────────────────────┘  ├─ chat_id(FK) │
                                              └──────────────┘
```

### Key Constraints

1. **Referential Integrity**: All foreign keys have `ON DELETE CASCADE` for data consistency
2. **Uniqueness**: User email is globally unique; one preference record per user
3. **Domain Constraints**: Difficulty levels restricted to 3 values; module numbers 1–4
4. **Check Constraints**: token_count > 0, rating >= 1 AND <= 5

---

## Part 3: Qdrant Vector Store Schema

### Collection: `textbook_chunks`

Qdrant mirrors the chunk table with vector similarity search capability.

```yaml
# Qdrant collection configuration
collection_name: "textbook_chunks"
vector_size: 1536                              # OpenAI text-embedding-3-small dimension
distance_metric: "Cosine"                      # Similarity metric

# Payload schema (searchable metadata)
payload_schema:
  chunk_id:
    type: "uuid"
    description: "Reference to chunks.id in Postgres"

  chapter_id:
    type: "text"
    description: "e.g., 'module3/chapter3-vslam'"

  module_number:
    type: "integer"
    description: "Module 1–4"

  section_name:
    type: "text"
    description: "Chapter section heading"

  heading:
    type: "text"
    description: "Immediate heading of chunk"

  difficulty_level:
    type: "text"
    enum: ["Beginner", "Intermediate", "Advanced"]

  keywords:
    type: "array"
    items: "text"
    description: "Searchable keywords"

  learning_objectives:
    type: "array"
    items: "text"

  url_anchor:
    type: "text"
    description: "Direct link to chunk on website"

  created_at:
    type: "datetime"

  token_count:
    type: "integer"

# Indexing configuration
indexes:
  - field: "module_number"
    type: "keyword"

  - field: "difficulty_level"
    type: "keyword"

  - field: "chapter_id"
    type: "keyword"

  - field: "keywords"
    type: "text"
```

### Synchronization Strategy

**One-way sync**: Postgres → Qdrant

1. **Create chunk** in Postgres (with embedding already computed via OpenAI API)
2. **Extract embedding** and metadata
3. **Insert point** into Qdrant with same ID
4. **Verify** 1:1 correspondence between tables

**Fallback**: If Qdrant is stale, Postgres is the source of truth. Rebuild Qdrant from scratch if needed.

---

## Part 4: Indexing Strategy

### Postgres Indexes (for metadata & fast lookups)

| Index | Table | Columns | Type | Purpose |
|-------|-------|---------|------|---------|
| `idx_chunks_module` | chunks | module_number | B-tree | Filter by module |
| `idx_chunks_difficulty` | chunks | difficulty_level | B-tree | Filter by difficulty |
| `idx_chunks_chapter_id` | chunks | chapter_id | B-tree | Filter by chapter |
| `idx_chunks_keywords` | chunks | keywords | GIN | Full-text keyword search |
| `idx_chunks_embedding` | chunks | embedding | ivfflat | Vector similarity (200K vectors) |
| `idx_chat_user_id` | chat_history | user_id | B-tree | User's chat history |
| `idx_chat_created` | chat_history | created_at | B-tree | Time-range queries (analytics) |
| `idx_feedback_type` | feedback | feedback_type | B-tree | Aggregate feedback by issue type |

### Expected Query Performance Targets

| Query Type | Expected Latency | Achieved Via |
|------------|------------------|--------------|
| Vector similarity (top-10 chunks) | 50–100 ms | Qdrant ivfflat index, nprobe=20 |
| Metadata filter + vector search | 100–200 ms | Combined Postgres + Qdrant search |
| User chat history (last 20) | <10 ms | B-tree on user_id + created_at |
| Feedback aggregation by type | <50 ms | B-tree index, GROUP BY query |
| Full-text keyword search | 20–50 ms | Postgres GIN index on keywords array |

---

## Part 5: Data Migration Strategy

### Initial Load (Phase 4)

1. **Extract chunks** from 15 markdown chapters (~2,500–3,000 total)
2. **Compute embeddings** via OpenAI API (batched, 20 req/min rate limit)
3. **Insert into Postgres** (`chunks` table)
4. **Sync to Qdrant** (upsert all points)
5. **Verify counts**: `chunks` table = Qdrant point count

**Estimated Time**: 2–4 hours (depends on API rate limits)

### Ongoing Migrations

#### Adding a New Chapter (Phase 3–13)

```bash
# 1. Extract chunks from new chapter markdown
python scripts/chunk_chapter.py docs/module4/chapter-new.md

# 2. Compute embeddings
python scripts/embed_chunks.py chunks_new.jsonl

# 3. Migrate to Postgres
python scripts/migrate_to_postgres.py chunks_new.jsonl

# 4. Sync to Qdrant
python scripts/sync_to_qdrant.py

# 5. Verify
SELECT COUNT(*) FROM chunks WHERE chapter_id = 'module4/chapter-new';
curl -X GET "http://localhost:6333/collections/textbook_chunks/points/count"
```

#### Schema Evolution (Add a New Metadata Field)

```sql
-- Example: Add "video_url" field
ALTER TABLE chunks ADD COLUMN video_url VARCHAR(512);

-- Backfill existing rows
UPDATE chunks SET video_url = NULL;

-- Create index if field is frequently filtered
CREATE INDEX idx_chunks_has_video ON chunks(video_url) WHERE video_url IS NOT NULL;
```

### Rollback Strategy

**If a migration introduces data inconsistency**:

1. **Stop all writes** to affected table
2. **Restore from Postgres backup** (Neon provides daily snapshots)
3. **Verify data integrity** (run checksums)
4. **Re-sync from Postgres** to Qdrant (rebuild from source of truth)
5. **Resume writes** once verified

**Backup Schedule**:
- Daily automated snapshots (Neon)
- Weekly manual snapshots to S3
- Point-in-time recovery available (30-day window)

---

## Part 6: Scaling & Performance Analysis

### Expected Data Volume

| Table | Records | Estimated Size | Notes |
|-------|---------|---|---|
| `chunks` | 2,500–3,000 | 500 MB | Text + 1536-dim vectors |
| `users` | 1,000–10,000 | <10 MB | Depends on adoption |
| `user_preferences` | 1,000–10,000 | <5 MB | 1:1 with users |
| `chat_history` | 10,000–100,000 | 100–500 MB | 10–100 chats per user |
| `feedback` | 1,000–10,000 | <20 MB | 5–20% of chats |

**Total Database Size**: ~700 MB – 1 GB (small enough for Neon Free tier)

### Scaling Strategy

| Threshold | Action |
|-----------|--------|
| >50K chats/month | Enable Postgres connection pooling (PgBouncer) |
| >100K users | Partition `chat_history` by `user_id` ranges |
| >500K chunks | Migrate to Qdrant Enterprise (unlimited collections) |
| Storage >50 GB | Archive old chat history (>1 year) to cold storage |

### Cost Estimate (First Year)

- **Neon Postgres**: $0–50/month (free tier, or pro at $0.30/CPU-hour)
- **Qdrant**: $0 (open-source, self-hosted) or $50+/month (managed)
- **OpenAI Embeddings**: $0.02 per 1K tokens
  - Initial load: ~10M tokens = $200
  - Monthly: ~5K new chunks = $1/month
- **Total**: $200–$300 (mostly one-time setup)

---

## Part 7: Data Consistency & Integrity Checks

### Validation Queries

```sql
-- Check for orphaned chat entries
SELECT COUNT(*) FROM chat_history
WHERE user_id NOT IN (SELECT id FROM users);
-- Expected: 0

-- Check for missing chunks in Qdrant
SELECT COUNT(*) FROM chunks
WHERE id NOT IN (SELECT chunk_id FROM qdrant_sync_log);
-- Expected: 0 (or list of chunks not yet synced)

-- Verify token counts are within bounds
SELECT COUNT(*) FROM chunks
WHERE token_count < 100 OR token_count > 1000;
-- Expected: <5% (outliers are acceptable)

-- Check for duplicate chapters
SELECT chapter_id, COUNT(*) FROM chunks
GROUP BY chapter_id HAVING COUNT(*) > 100;
-- Expected: All chunks are unique (no more than ~300 per chapter)
```

### Data Quality SLOs

| Metric | Target | Monitoring |
|--------|--------|------------|
| Chunk embedding coverage | 100% | Daily check (SELECT COUNT(*) WHERE embedding IS NULL) |
| User preferences consistency | 100% | Foreign key constraints (automatic) |
| Chat history retention | ≥2 years | Automated archive job (monthly) |
| Citation accuracy | ≥95% | Manual QA feedback sampling (bi-weekly) |

---

## Summary

This schema supports:

✅ **Fast RAG queries** via Qdrant vector similarity + Postgres metadata filtering
✅ **User personalization** via preferences table
✅ **Feedback collection** for model improvement
✅ **Audit trails** (chat history, timestamps)
✅ **Scalability** from 1K to 100K+ users
✅ **Data integrity** via referential constraints and checksums

**Next Phase**: Phase 4 will implement the actual migration and embedding generation pipeline.
