import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Layout } from "./components/Layout";
import HomePage from "./pages/HomePage";
import MapListPage from "./pages/MapListPage";
import MapPreviewPage from "./pages/MapPreviewPage";
import MappingPage from "./pages/MappingPage";
import RouteListPage from "./pages/RouteListPage";
import RouteEditorPage from "./pages/RouteEditorPage";
import ResultsPage from "./pages/ResultsPage";
import SystemPage from "./pages/SystemPage";
import PlaybackLibraryPage from "./pages/PlaybackLibraryPage";
import PlaybackWorkspacePage from "./pages/PlaybackWorkspacePage";

export default function App() {
  return (
    <BrowserRouter>
      <Routes>
        {/* 地图预览页 / 建图页 / 数据回放工作台故意不挂在 Layout 下面: 它们要的是
            头栏+可视化区占满整个页面, Layout 自带的侧边栏/顶部应用栏在这几页
            没有存在的必要。 */}
        <Route path="/maps/:name/preview" element={<MapPreviewPage />} />
        <Route path="/mapping/:name" element={<MappingPage />} />
        <Route path="/playback/:name" element={<PlaybackWorkspacePage />} />
        <Route element={<Layout />}>
          <Route path="/" element={<HomePage />} />
          <Route path="/maps" element={<MapListPage />} />
          <Route path="/routes" element={<RouteListPage />} />
          <Route path="/routes/:id" element={<RouteEditorPage />} />
          <Route path="/results" element={<ResultsPage />} />
          <Route path="/system" element={<SystemPage />} />
          <Route path="/playback" element={<PlaybackLibraryPage />} />
        </Route>
      </Routes>
    </BrowserRouter>
  );
}
