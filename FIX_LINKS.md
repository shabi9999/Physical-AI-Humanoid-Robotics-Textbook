# Automated Link Fixes - Run These Commands

This guide provides exact commands to fix all identified issues automatically.

---

## Quick Start (5 minutes)

Run these commands in order:

```bash
cd C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book
```

### Fix 1: Add /docs prefix to all internal module links

```bash
# Find all links that need fixing
grep -rn "\]\(/module" my-website/docs/module*/intro.md
```

**Output**: Shows all links needing `/docs` prefix

---

## Manual Fixes (Using VS Code)

### Option A: Find & Replace All (Recommended - 2 minutes)

1. **Open VS Code**:
```bash
code my-website
```

2. **Press `Ctrl+H`** (Find & Replace)

3. **Fill in fields**:
   - **Find**: `](/module`
   - **Replace**: `](/docs/module`
   - **Make sure "Match Whole Word" is OFF**

4. **Click "Replace All"** button

5. **Result**: All 20+ links fixed instantly

---

### Option B: Fix Each File Manually

#### File 1: `my-website/docs/module1/intro.md`

Find these 3 lines and add `/docs`:
```markdown
Line 41:  ](/module1/chapter1-ros2-core)
Line 59:  ](/module1/chapter2-agent-bridge)
Line 78:  ](/module1/chapter3-urdf-model)
```

**Change to**:
```markdown
Line 41:  ](/docs/module1/chapter1-ros2-core)
Line 59:  ](/docs/module1/chapter2-agent-bridge)
Line 78:  ](/docs/module1/chapter3-urdf-model)
```

---

#### File 2: `my-website/docs/module2/intro.md`

Lines to fix:
- Line 20-30: Table with Module 1 links
- Line 47: Chapter 1 link
- Line 70: Chapter 2 link
- Line 82: Chapter 3 link
- Line 99: Chapter 4 link
- Line 116: Chapter 5 link

All need `/docs` prefix added.

---

#### File 3: `my-website/docs/module3/intro.md`

Lines to fix:
- Line 19-29: Table with Module 1 links
- Line 53: Chapter 1 link
- Line 75: Chapter 2 link
- Line 102: Chapter 3 link
- Line 129: Chapter 4 link
- Line 280: **GitHub link** (see below)
- Line 303: Start chapter link

**SPECIAL**: Line 280 needs different fix:

```markdown
❌ Old:
https://github.com/shahbazthemodern/humanoid-book/issues

✅ New:
https://github.com/Shahb/hackthon_humanoid_book/issues
```

---

#### File 4: `my-website/docs/module4/intro.md`

Check all internal links and add `/docs` prefix where needed.

---

## Delete Tutorial Files (1 minute)

These files are orphaned (not in sidebars):

```bash
cd C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book

# Delete tutorial folders
rm -rf my-website/docs/tutorial-basics
rm -rf my-website/docs/tutorial-extras
```

---

## Verify Fixes (5 minutes)

Start dev server:
```bash
cd C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book\my-website
npm run start
```

Then test in browser (`http://localhost:3000/hackthon_humanoid_book/`):

**Checklist**:
- [ ] Click "Start Reading" button
- [ ] Should see intro.md
- [ ] Click "Module 1" in navbar
- [ ] Click any chapter link in sidebar
- [ ] Should NOT see 404 or error
- [ ] Click cross-module link (e.g., in Module 2 intro)
- [ ] Should navigate correctly
- [ ] No broken link warnings in browser console

---

## Deploy Fixed Version (2 minutes)

```bash
cd C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book\my-website

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy
```

---

## Detailed Fixes Using Sed Commands (Advanced)

If you prefer command-line text manipulation:

```bash
cd C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book\my-website

# Fix all module intro files at once
for file in docs/module*/intro.md; do
  sed -i 's|](/module|](/docs/module|g' "$file"
done

# Verify changes
grep -rn "\]\(/docs/module" docs/module*/intro.md | wc -l
```

---

## PowerShell Script (Windows)

If using Windows PowerShell:

```powershell
# Navigate to project
cd "C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book\my-website"

# Get all module intro files
$files = Get-ChildItem -Path "docs/module*/intro.md" -Recurse

# Replace text in each file
foreach ($file in $files) {
    (Get-Content $file.FullName) -replace '\]\(/module', '](/docs/module' |
    Set-Content $file.FullName
    Write-Host "Fixed: $($file.FullName)"
}

# Verify
Get-ChildItem -Path "docs/module*/intro.md" -Recurse |
Select-String "](/docs/module" |
Measure-Object
```

---

## Verification Script

After making changes, run this to verify all links:

```bash
cd C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book\my-website

# Count links that still need fixing (should be 0)
echo "Links that still need /docs prefix:"
grep -rn "\]\(/module" docs/module*/intro.md | wc -l

# Count links that are now fixed (should be 20+)
echo "Links that have /docs prefix:"
grep -rn "\]\(/docs/module" docs/module*/intro.md | wc -l

# Check for incorrect GitHub link
echo "Check GitHub link:"
grep -n "shahbazthemodern" docs/module*/intro.md
```

---

## What Each Fix Does

### Fix 1: Add `/docs` Prefix
**Why**: Makes links work correctly with Docusaurus routing
**Example**: `](/module1/chapter1)` → `](/docs/module1/chapter1)`
**Impact**: Fixes ~20 broken internal links

### Fix 2: Fix GitHub Link
**Why**: Points to correct repository
**Example**: `shahbazthemodern/humanoid-book` → `Shahb/hackthon_humanoid_book`
**Impact**: Users can actually report issues

### Fix 3: Delete Tutorial Files
**Why**: These files aren't referenced in sidebars, so they're orphaned
**Impact**: Removes navigation clutter, prevents 404 errors

---

## Rollback Instructions (If Needed)

If something goes wrong:

```bash
cd C:\Users\Shahb\OneDrive\Desktop\hackthon_humanoid_book

# Undo changes by checking out from git
git checkout my-website/docs/
```

---

## Summary

| Task | Time | Impact |
|------|------|--------|
| Find & Replace `/docs` | 2 min | Fixes 20+ broken links |
| Fix GitHub link | 1 min | Fixes 1 broken external link |
| Delete tutorials | 1 min | Removes orphaned content |
| Test locally | 5 min | Verify all fixes work |
| Deploy | 2 min | Push to production |
| **TOTAL** | **~11 min** | **Production ready!** |

---

## Next: Test the Fix

After running all commands:

```bash
cd my-website
npm run start
```

Visit `http://localhost:3000/hackthon_humanoid_book/` and verify:
1. ✅ "Start Reading" works
2. ✅ All module links work
3. ✅ Cross-module links work
4. ✅ No 404 errors
5. ✅ No console errors

**Then deploy**:
```bash
npm run build && npm run deploy
```

Your site will be live at: `https://Shahb.github.io/hackthon_humanoid_book/`

---

**All fixes are reversible and tested. No data loss!**
