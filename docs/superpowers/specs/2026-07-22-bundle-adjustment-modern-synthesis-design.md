# Design: Bundle Adjustment — A Modern Synthesis

## Goal

Publish a Chinese long-form blog post for readers with prior multi-view geometry knowledge. The post will explain why Triggs et al.'s 2000 survey was needed, distil its most consequential ideas, and connect them to contemporary SfM and SLAM practice. It is an interpretive tutorial, not a chapter-by-chapter translation.

## Audience and scope

- Audience: graduate students and engineers who know epipolar geometry, triangulation, and nonlinear optimization at a basic level.
- Depth: derive the reprojection objective and explain block sparsity and Schur complement conceptually; discuss robust estimation, gauge freedom, uncertainty, and network design without reproducing every proof in the paper.
- Exclusions: no implementation walkthrough for a specific library, no claim that the article replaces the original paper, and no unlicensed reproduction of paper figures.

## Narrative structure

1. **The historical problem** — Explain the gap between mature photogrammetric adjustment theory and the computer-vision community's fragmented, heuristic reconstruction iterations around 2000.
2. **What BA optimizes** — Define the joint camera-and-structure state, projection model, residuals, and robust objective. Contrast joint global consistency with pairwise estimation and triangulation.
3. **Why BA scales** — Show the observation graph, the camera/point block Hessian, and the Schur complement intuition. Explain why sparse second-order methods, rather than generic dense optimization, are central to the paper.
4. **What makes it statistically trustworthy** — Cover robust costs, noise weighting, local parameterizations, gauge/datum ambiguity, covariance, residual checks, and redundancy.
5. **Why the measurement network matters** — Explain degeneracy, viewing baselines, overlap, self-calibration, and why optimization cannot recover information missing from acquisition.
6. **Modern reading** — Place BA after initialization in SfM and as a recurring local/global back end in SLAM; state where it remains relevant and what modern systems add.
7. **Takeaways and references** — End with a concise practical checklist and bibliographic references to the source paper, textbook background, and relevant modern optimization software.

## Visual and mathematical plan

- Use MathJax display equations for the robust joint objective, Gauss–Newton linearization, block normal equations, and Schur complement.
- Create original explanatory diagrams: an end-to-end measurement-to-BA flow and a sparse camera/point observation graph or block-matrix diagram.
- Attribute the paper and quote no extended passages. Do not copy its figures; use original diagrams instead.

## Publication plan

- Write one Markdown source file on the desktop, then publish with `upload_pages.py` using `--mode post` and the tags `Computer Vision`, `SLAM`, and `Bundle Adjustment`.
- Let the publishing workflow add blog frontmatter and date.
- Verify the pushed GitHub Actions deployment, the direct post URL, and HTTP 200 before reporting publication success.

## Acceptance criteria

- The article explains the historical meaning of “modern synthesis” before technical detail.
- It gives an accurate, readable account of the paper's major technical contributions and clearly distinguishes BA from multi-view geometry generally.
- Formulas render in VitePress/MathJax and every diagram is original, valid, and captioned.
- The blog post is committed, pushed, reachable, and its deployed page passes the required verification.
