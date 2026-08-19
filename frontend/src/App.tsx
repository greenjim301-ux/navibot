import { BrowserRouter, Routes, Route } from "react-router-dom";
import { Layout } from "./components/Layout";
import MapListPage from "./pages/MapListPage";
import MapPreviewPage from "./pages/MapPreviewPage";
import NavigatePage from "./pages/NavigatePage";
import RouteListPage from "./pages/RouteListPage";
import RouteCreatePage from "./pages/RouteCreatePage";
import RoutePreviewPage from "./pages/RoutePreviewPage";

export default function App() {
  return (
    <BrowserRouter>
      <Routes>
        {/* 地图预览页故意不挂在 Layout 下面: 它要的是头栏+预览内容占满整个页面,
            Layout 自带的侧边栏/顶部应用栏在这页没有存在的必要。 */}
        <Route path="/maps/:name/preview" element={<MapPreviewPage />} />
        <Route element={<Layout />}>
          <Route path="/" element={<MapListPage />} />
          <Route path="/maps/:name/navigate" element={<NavigatePage />} />
          <Route path="/routes" element={<RouteListPage />} />
          <Route path="/routes/new" element={<RouteCreatePage />} />
          <Route path="/routes/:id" element={<RoutePreviewPage />} />
        </Route>
      </Routes>
    </BrowserRouter>
  );
}
