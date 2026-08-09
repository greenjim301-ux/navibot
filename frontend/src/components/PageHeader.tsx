import type { ReactNode } from "react";
import { Link } from "react-router-dom";

interface Props {
  /** 返回上一级的链接, 不传就不显示 */
  backTo?: string;
  backLabel?: string;
  title: ReactNode;
  /** 一句话说明, 跟标题排在同一行, 窄屏才换行 */
  description?: ReactNode;
  /** 右侧操作区 */
  actions?: ReactNode;
}

/**
 * 各页统一的紧凑标题栏: 返回链接 + 标题 + 说明 + 右侧操作全部排在一行,
 * 尽量少占正文空间 (之前是"小字标签/大标题/说明"三行, 太占地方)。
 */
export function PageHeader({ backTo, backLabel, title, description, actions }: Props) {
  return (
    <header className="mb-5 flex flex-wrap items-center gap-x-3 gap-y-1">
      {backTo && (
        <Link
          to={backTo}
          className="shrink-0 text-[13px] text-muted-foreground hover:text-foreground"
        >
          ← {backLabel ?? "返回"}
        </Link>
      )}
      <h1 className="flex shrink-0 items-center gap-2 text-lg font-semibold tracking-tight">
        {title}
      </h1>
      {description && (
        <p className="min-w-0 truncate text-[13px] text-muted-foreground">{description}</p>
      )}
      {actions && <div className="ml-auto flex shrink-0 items-center gap-2">{actions}</div>}
    </header>
  );
}
