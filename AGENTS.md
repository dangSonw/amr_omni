<!-- gitnexus:start -->
# GitNexus — Code Intelligence

This project is indexed by GitNexus as **amr_omni** (7029 symbols, 16916 relationships, 300 execution flows). Use the GitNexus MCP tools to understand code, assess impact, and navigate safely.

> Index stale? Run `node .gitnexus/run.cjs analyze` from the project root — it auto-selects an available runner. No `.gitnexus/run.cjs` yet? `npx gitnexus analyze` (npm 11 crash → `npm i -g gitnexus`; #1939).

## Always Do

- **MUST run impact analysis before editing any symbol.** Before modifying a function, class, or method, run `impact({target: "symbolName", direction: "upstream"})` and report the blast radius (direct callers, affected processes, risk level) to the user.
- **MUST run `detect_changes()` before committing** to verify your changes only affect expected symbols and execution flows. For regression review, compare against the default branch: `detect_changes({scope: "compare", base_ref: "main"})`.
- **MUST warn the user** if impact analysis returns HIGH or CRITICAL risk before proceeding with edits.
- When exploring unfamiliar code, use `query({search_query: "concept"})` to find execution flows instead of grepping. It returns process-grouped results ranked by relevance.
- When you need full context on a specific symbol — callers, callees, which execution flows it participates in — use `context({name: "symbolName"})`.
- For security review, `explain({target: "fileOrSymbol"})` lists taint findings (source→sink flows; needs `analyze --pdg`).

## Never Do

- NEVER edit a function, class, or method without first running `impact` on it.
- NEVER ignore HIGH or CRITICAL risk warnings from impact analysis.
- NEVER rename symbols with find-and-replace — use `rename` which understands the call graph.
- NEVER commit changes without running `detect_changes()` to check affected scope.

## Resources

| Resource | Use for |
|----------|---------|
| `gitnexus://repo/amr_omni/context` | Codebase overview, check index freshness |
| `gitnexus://repo/amr_omni/clusters` | All functional areas |
| `gitnexus://repo/amr_omni/processes` | All execution flows |
| `gitnexus://repo/amr_omni/process/{name}` | Step-by-step execution trace |

## CLI

| Task | Read this skill file |
|------|---------------------|
| Understand architecture / "How does X work?" | `.agents/skills/gitnexus/gitnexus-exploring/SKILL.md` |
| Blast radius / "What breaks if I change X?" | `.agents/skills/gitnexus/gitnexus-impact-analysis/SKILL.md` |
| Trace bugs / "Why is X failing?" | `.agents/skills/gitnexus/gitnexus-debugging/SKILL.md` |
| Rename / extract / split / refactor | `.agents/skills/gitnexus/gitnexus-refactoring/SKILL.md` |
| Tools, resources, schema reference | `.agents/skills/gitnexus/gitnexus-guide/SKILL.md` |
| Index, status, clean, wiki CLI commands | `.agents/skills/gitnexus/gitnexus-cli/SKILL.md` |

<!-- gitnexus:end -->

---

# Mandatory Rules for AI Agents (Quy tắc bắt buộc cho AI Agent)

## 1. BẮT BUỘC dùng GitNexus để đọc và điều hướng dự án
- **Đọc hiểu dự án:** Tuyệt đối KHÔNG grep/find bừa bãi khi cần hiểu kiến trúc hay luồng code. PHẢI dùng `query({search_query: "concept"})` hoặc MCP tools tương ứng để tìm các execution flow được xếp hạng theo độ liên quan.
- **Xem ngữ cảnh symbol:** Dùng `context({name: "symbolName"})` để lấy đầy đủ callers, callees và các execution flow liên quan.
- **Phân tích tác động (Impact Analysis):** BẮT BUỘC chạy `impact({target: "symbolName", direction: "upstream"})` trước khi sửa bất kỳ hàm, class, hoặc method nào. Báo cáo blast radius và mức độ rủi ro (risk level). CẢNH BÁO người dùng nếu mức độ là HIGH hoặc CRITICAL.
- **Kiểm tra trước khi commit:** BẮT BUỘC chạy `detect_changes()` trước khi commit code để đảm bảo thay đổi chỉ ảnh hưởng đúng phạm vi mong muốn.

## 2. BẮT BUỘC tuân thủ thiết kế và quy tắc các Agent Skills (`.agents/skills/`)
Mọi AI Agent khi làm việc trong repository `amr_omni` PHẢI tuân thủ triệt để các bộ quy tắc skill sau:
- **`ponytail` (Code tối giản & Tinh gọn - YAGNI):** 
  - Ưu tiên giải pháp đơn giản nhất, ngắn nhất, tối thiểu mà vẫn chạy tốt.
  - Tái sử dụng code/helper/hàm có sẵn trong codebase trước khi viết mới.
  - Không thêm abstraction không cần thiết (không tạo wrapper, factory, config cho giá trị không đổi).
  - Không viết code "dành cho tương lai". Diff ngắn nhất, rõ ràng nhất, sửa tận gốc (root cause).
- **`caveman` (Giao tiếp súc tích):**
  - Trả lời ngắn gọn, trực diện, bỏ từ ngữ rườm rà, filler, mào đầu hay kết thúc sáo rỗng.
  - Giữ lại đầy đủ tính chính xác kỹ thuật, tên lệnh, code, lỗi và số liệu.
- **`headroom` (Tối ưu Context & Nén Token):**
  - Tự động nén/tinh giản các log build dài, mảng JSON lớn, kết quả grep dài trước khi đưa vào context để tránh tràn context window và giảm chi phí token.
- **`frontend-design`, `web-design-guidelines`, `ui-skills` (Chuẩn thiết kế Web & UI):**
  - Giao diện web dashboard phải tuân thủ phong cách công nghiệp/flat (bo góc tối thiểu `rounded-sm`), nhất quán màu sắc/token, bố cục rõ ràng, phản hồi nhanh, trực quan và chuyên nghiệp.
  - Nghiêm cấm tạo giao diện cẩu thả, màu sắc lòe loẹt, layout vỡ hoặc thiếu trạng thái tải/lỗi.
- **`vercel-react-best-practices` (Hiệu năng React & Next.js):**
  - Tuân thủ 45 quy tắc tối ưu hiệu năng của Vercel cho React và Next.js.
  - Triệt tiêu waterfalls (dùng parallel fetch, defer await), tối ưu kích thước bundle, tránh re-render thừa (functional setState, memo, derived state), tối ưu hydration.
- **`find-skills` & `skill-creator`:**
  - Sử dụng `find-skills` khi cần tìm kiếm thêm công cụ/skill mở rộng.
  - Tuân thủ cấu trúc chuẩn khi tạo skill mới (YAML frontmatter, tài liệu rõ ràng, kịch bản kiểm thử).
