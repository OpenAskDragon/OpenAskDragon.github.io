# Bundle Adjustment — A Modern Synthesis Blog Post Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a detailed Chinese blog post that explains the historical purpose, technical insights, and modern relevance of Triggs et al.'s *Bundle Adjustment — A Modern Synthesis*.

**Architecture:** Extract primary-source facts from the supplied PDF into a tracked evidence outline. Write a self-contained Markdown post with MathJax and two original SVG explanations, then use the site's publishing script to place it under `posts/`, commit, push, and verify its deployed route.

**Tech Stack:** Conda Python with PyMuPDF/pypdf for PDF extraction; Markdown and MathJax; inline SVG assets; VitePress; GitHub Actions; `upload_pages.py`.

## Global Constraints

- Write for readers who already know epipolar geometry, triangulation, and basic nonlinear optimization.
- Explain the historical motivation before the technical material; do not make a section-by-section translation.
- Use original diagrams only; do not reproduce figures from the paper.
- Put all displayed mathematics in MathJax-compatible `$$...$$` blocks.
- Publish in post mode with the tags `Computer Vision`, `SLAM`, and `Bundle Adjustment`.
- Report publication success only after the workflow and deployed direct route have been verified.

---

### Task 1: Build a source-grounded evidence outline

**Files:**
- Read: `C:\Users\I\Desktop\Triggs00.pdf`
- Read: `C:\Users\I\Desktop\bundleadjustment.pdf`
- Create: `C:\Users\I\.claude\repos\openaskdragon.github.io\docs\superpowers\evidence\2026-07-22-bundle-adjustment-modern-synthesis.md`

**Interfaces:**
- Consumes: the two user-supplied PDF variants.
- Produces: a compact, attributed outline of the paper's abstract, historical motivation, methods, recommendations, and bibliography data for Task 2.

- [ ] **Step 1: Extract selected text using the available Conda/Python PDF packages**

Run:

```powershell
conda run -n base python -c "import fitz; d=fitz.open(r'C:\Users\I\Desktop\Triggs00.pdf'); print(len(d))"
```

Expected: the command reports a readable, nonzero page count.

- [ ] **Step 2: Compare title, authors, page count, and abstract across both variants**

Record that both PDFs are versions of the same Triggs, McLauchlan, Hartley, and Fitzgibbon paper, and identify any pagination difference only as a document-version detail.

- [ ] **Step 3: Write the evidence outline**

Include pointers to the source paper's introduction, coverage statement, and summary/recommendations, along with the article's own mathematical and interpretive claims to be supported in Task 2.

- [ ] **Step 4: Validate the outline**

Run:

```powershell
rg -n "historical|robust|sparse|gauge|quality|network" C:\Users\I\.claude\repos\openaskdragon.github.io\docs\superpowers\evidence\2026-07-22-bundle-adjustment-modern-synthesis.md
```

Expected: every required theme has a sourced outline entry.

### Task 2: Write the standalone long-form article

**Files:**
- Read: `C:\Users\I\.claude\repos\openaskdragon.github.io\docs\superpowers\evidence\2026-07-22-bundle-adjustment-modern-synthesis.md`
- Create: `C:\Users\I\.claude\repos\openaskdragon.github.io\docs\superpowers\drafts\bundle-adjustment-modern-synthesis.md`

**Interfaces:**
- Consumes: the factual evidence outline from Task 1 and the original diagrams from Task 3.
- Produces: a UTF-8 Markdown post with a single H1 title, MathJax equations, image links, captions, source attribution, and references.

- [ ] **Step 1: Write the historical opening and thesis**

Explain that BA was mature in photogrammetry and geodesy, while computer-vision practice often treated reconstruction refinement as disconnected heuristics. State that “modern synthesis” means joining statistical estimation, sparse numerical linear algebra, geometry, and reliability practice for CV implementers.

- [ ] **Step 2: Add the joint estimation model**

Include the robust reprojection objective, define observations, cameras, structure, projection, covariances, robust loss, and priors, and explain why pairwise geometry plus triangulation is not jointly optimal.

- [ ] **Step 3: Add the sparse optimization deep dive**

Derive the local Gauss–Newton block system and the Schur complement conceptually. Explain the camera-point bipartite observation graph and why a generic dense optimizer creates the misleading impression that BA is slow.

- [ ] **Step 4: Add the statistical and geometric deep dive**

Cover robust losses and weighting, local rotation/parameter updates, gauge freedom/datum choice, uncertainty and quality control, and image-network design. Explain explicitly that optimization cannot repair weak baselines or missing redundancy.

- [ ] **Step 5: Add the modern interpretation and references**

Map initialization, local BA, global BA, and sliding-window BA to modern SfM/SLAM systems. End with a practical checklist and bibliographic entries for Triggs et al. (2000), Hartley and Zisserman (2004), Nocedal and Wright (2006), Agarwal et al. (2010), and Ceres Solver.

- [ ] **Step 6: Validate Markdown structure and mathematical delimiters**

Run:

```powershell
rg -n "^#|^##|^\$\$|^!\[|^## 参考文献" C:\Users\I\.claude\repos\openaskdragon.github.io\docs\superpowers\drafts\bundle-adjustment-modern-synthesis.md
```

Expected: one H1, all planned H2 sections, four display-math delimiter pairs, two figure links, and a references heading.

### Task 3: Create and validate original explanatory figures

**Files:**
- Create: `C:\Users\I\.claude\repos\openaskdragon.github.io\public\images\bundle-adjustment-pipeline.svg`
- Create: `C:\Users\I\.claude\repos\openaskdragon.github.io\public\images\bundle-adjustment-schur.svg`

**Interfaces:**
- Consumes: the article concepts from Task 2.
- Produces: two original SVGs addressable by VitePress as `/images/bundle-adjustment-pipeline.svg` and `/images/bundle-adjustment-schur.svg`.

- [ ] **Step 1: Draw the measurement-to-optimization pipeline**

Depict images, feature tracks, initialization, joint robust optimization, and refined cameras/3D points. Caption it in the Markdown as an original schematic.

- [ ] **Step 2: Draw the sparse block-system schematic**

Depict camera blocks, point blocks, observation couplings, and elimination of point increments into the reduced camera system. Caption it as the Schur-complement intuition rather than an exact implementation diagram.

- [ ] **Step 3: Validate SVG syntax and asset references**

Run:

```powershell
[xml](Get-Content -LiteralPath 'C:\Users\I\.claude\repos\openaskdragon.github.io\public\images\bundle-adjustment-pipeline.svg' -Raw) | Out-Null
[xml](Get-Content -LiteralPath 'C:\Users\I\.claude\repos\openaskdragon.github.io\public\images\bundle-adjustment-schur.svg' -Raw) | Out-Null
```

Expected: both commands exit successfully.

### Task 4: Publish and verify the blog post

**Files:**
- Read: `C:\Users\I\.claude\repos\openaskdragon.github.io\docs\superpowers\drafts\bundle-adjustment-modern-synthesis.md`
- Modify: `C:\Users\I\.claude\repos\openaskdragon.github.io\posts\2026-07-22-*.md` (created by the upload script)
- Modify: `C:\Users\I\.claude\repos\openaskdragon.github.io\public\images\bundle-adjustment-pipeline.svg`
- Modify: `C:\Users\I\.claude\repos\openaskdragon.github.io\public\images\bundle-adjustment-schur.svg`

**Interfaces:**
- Consumes: the final Markdown and validated assets.
- Produces: one pushed post and a public route under `https://openaskdragon.github.io/posts/`.

- [ ] **Step 1: Run the required publishing script in post mode**

Run:

```powershell
python C:\Users\I\.codex\skills\upload-pages\scripts\upload_pages.py C:\Users\I\.claude\repos\openaskdragon.github.io\docs\superpowers\drafts\bundle-adjustment-modern-synthesis.md --mode post --date 2026-07-22 -t 'Computer Vision' 'SLAM' 'Bundle Adjustment' -m 'docs: publish bundle adjustment modern synthesis'
```

Expected: the script copies the article to `posts/`, commits all staged post and SVG assets, pushes `master`, and prints the post URL.

- [ ] **Step 2: Build the site locally**

Run:

```powershell
npm run docs:build
```

Expected: VitePress completes without a broken-link or Markdown rendering error.

- [ ] **Step 3: Verify deployment and the direct route**

Run GitHub CLI or the GitHub Actions web API for the pushed commit until the workflow succeeds, then request the direct post URL and require HTTP 200.

- [ ] **Step 4: Commit any build-related fixes if validation required them**

Run:

```powershell
git status --short
```

Expected: clean worktree, unless a verified rendering fix is needed; in that case commit and push the targeted fix before repeating Step 3.
