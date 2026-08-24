import { useMemo, useState } from "react";
import { Link } from "react-router-dom";
import { Plus, RefreshCw } from "lucide-react";
import { FAKE_DATASETS } from "../data/fakeDatasets";
import { ReplayCloudStage } from "../components/ReplayCloudStage";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import {
  Card, CardHeader, CardTitle, CardDescription, CardFooter,
} from "@/components/ui/card";

export default function PlaybackLibraryPage() {
  const [search, setSearch] = useState("");

  const filtered = useMemo(() => {
    const q = search.trim().toLowerCase();
    return q ? FAKE_DATASETS.filter((d) => d.name.toLowerCase().includes(q)) : FAKE_DATASETS;
  }, [search]);

  return (
    <div className="px-8 py-6">
      <div className="mb-1 text-xs text-muted-foreground">
        数据中心<span className="mx-1.5">/</span>DATA PLAYBACK
      </div>
      <div className="mb-5 flex flex-wrap items-start justify-between gap-3">
        <div>
          <h1 className="text-2xl font-bold tracking-tight">数据回放</h1>
          <p className="mt-1 text-sm text-muted-foreground">选择采集数据集，进入点云帧回放</p>
        </div>
        <div className="flex gap-2">
          <Button variant="outline" disabled title="演示数据，暂不支持刷新">
            <RefreshCw />
            刷新
          </Button>
          <Button disabled title="演示数据，暂不支持上传">
            <Plus />
            上传数据
          </Button>
        </div>
      </div>

      <div className="mb-6 flex items-center gap-3">
        <Input
          placeholder="搜索数据集名称"
          value={search}
          onChange={(e) => setSearch(e.target.value)}
          className="w-64"
        />
        <span className="text-xs text-muted-foreground">共 {FAKE_DATASETS.length} 组数据</span>
      </div>

      {filtered.length === 0 ? (
        <div className="rounded-xl border border-dashed py-16 text-center text-sm text-muted-foreground">
          没有匹配的数据集
        </div>
      ) : (
        <div className="grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {filtered.map((d) => (
            <Card key={d.name} className="gap-0 overflow-hidden py-0">
              <div className="group relative h-48">
                <ReplayCloudStage
                  seed={d.name}
                  frame={0}
                  pointColorMode="深度"
                  pointSize={1.5}
                  showRawCloud
                  compact
                  className="size-full"
                />
                <div className="absolute inset-0 flex items-center justify-center bg-black/45 opacity-0 transition-opacity group-hover:opacity-100">
                  <Button asChild size="sm" className="shadow-md">
                    <Link to={`/playback/${encodeURIComponent(d.name)}`}>进入回放</Link>
                  </Button>
                </div>
              </div>

              <CardHeader className="pt-4">
                <CardTitle className="truncate text-base">{d.name}</CardTitle>
                <CardDescription>{d.frames.toLocaleString()} 帧点云 · PKL 回放数据</CardDescription>
              </CardHeader>

              <CardFooter className="justify-between">
                <span className={`text-xs ${d.active ? "text-success" : "text-muted-foreground"}`}>
                  {d.active ? "● 当前使用" : "可回放"}
                </span>
                <Button variant="link" size="sm" className="h-auto p-0" asChild>
                  <Link to={`/playback/${encodeURIComponent(d.name)}`}>进入回放 →</Link>
                </Button>
              </CardFooter>
            </Card>
          ))}
        </div>
      )}
    </div>
  );
}
