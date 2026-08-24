// 数据回放页面目前是纯前端演示: 数据集列表和点云帧内容都是假数据, 真正接
// 后端(采集数据集存储/真实 PKL 点云帧读取)是后续单独的活。

export interface Dataset {
  name: string;
  frames: number;
  active: boolean;
}

export const FAKE_DATASETS: Dataset[] = [
  { name: "demo_data", frames: 9588, active: true },
  { name: "factory_patrol_0818", frames: 6240, active: false },
  { name: "warehouse_localization", frames: 3812, active: false },
];
