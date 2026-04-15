import { useState, useCallback, useMemo, useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Line, Text } from "@react-three/drei";
import { useTopicSubscription } from "../../hooks/useTopicSubscription";
import type { TFMessage } from "../../types/ros";
import { useRobotSelectorStore } from "../../stores/robot-selector-store";
import { Box, Eye, EyeOff, List, Bone } from "lucide-react";
import clsx from "clsx";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";

interface FrameData {
  parent: string;
  child: string;
  translation: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number; w: number };
  lastUpdate: number;
}

/** World-space pose computed by chaining parent transforms */
interface WorldPose {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
}

/**
 * Mapping from TF frame name to STL mesh path (served from /meshes/).
 * Frames without an entry render as axes + spheres.
 */
const LINK_MESHES: Record<string, { stl: string; rpy?: [number, number, number]; scale?: number }> = {
  // Meca500 arm
  meca_base_link:    { stl: "/meshes/meca500/meca_500_r3_base.stl" },
  meca_axis_1_link:  { stl: "/meshes/meca500/meca_500_r3_j1.stl" },
  meca_axis_2_link:  { stl: "/meshes/meca500/meca_500_r3_j2.stl" },
  meca_axis_3_link:  { stl: "/meshes/meca500/meca_500_r3_j3.stl" },
  meca_axis_4_link:  { stl: "/meshes/meca500/meca_500_r3_j4.stl" },
  meca_axis_5_link:  { stl: "/meshes/meca500/meca_500_r3_j5.stl" },
  meca_axis_6_link:  { stl: "/meshes/meca500/meca_500_r3_j6.stl" },
  // Schunk MEGp25e gripper
  meca_gripper_link:    { stl: "/meshes/meca500/schunk_megp25e_body.stl",   rpy: [1.5708, 0, 0], scale: 0.001 },
  meca_gripper_finger1: { stl: "/meshes/meca500/schunk_megp25e_finger.stl", scale: 0.001 },
  meca_gripper_finger2: { stl: "/meshes/meca500/schunk_megp25e_finger.stl", scale: 0.001 },
};

/**
 * Walk the TF tree from root to each frame, accumulating transforms
 * to produce world-space positions and orientations.
 */
function computeWorldPoses(frames: Map<string, FrameData>): Map<string, WorldPose> {
  const worldPoses = new Map<string, WorldPose>();

  function getWorldPose(frameName: string): WorldPose {
    if (worldPoses.has(frameName)) return worldPoses.get(frameName)!;

    const frame = frames.get(frameName);
    if (!frame) {
      const pose = {
        position: new THREE.Vector3(0, 0, 0),
        quaternion: new THREE.Quaternion(0, 0, 0, 1),
      };
      worldPoses.set(frameName, pose);
      return pose;
    }

    const parentPose = getWorldPose(frame.parent);

    const localPos = new THREE.Vector3(
      frame.translation.x,
      frame.translation.y,
      frame.translation.z
    );
    const localQuat = new THREE.Quaternion(
      frame.rotation.x,
      frame.rotation.y,
      frame.rotation.z,
      frame.rotation.w
    );

    const worldPos = localPos.clone().applyQuaternion(parentPose.quaternion).add(parentPose.position);
    const worldQuat = parentPose.quaternion.clone().multiply(localQuat);

    const pose = { position: worldPos, quaternion: worldQuat };
    worldPoses.set(frameName, pose);
    return pose;
  }

  for (const frameName of frames.keys()) {
    getWorldPose(frameName);
  }

  return worldPoses;
}

/**
 * Seed static transforms for fixed joints that rosbridge often misses
 * (/tf_static uses TRANSIENT_LOCAL durability — late-joining websocket
 * subscribers may never receive them). Live /tf_static data overrides
 * these when it arrives.
 */
function buildInitialFrames(): Map<string, FrameData> {
  const now = Date.now();
  const frames = new Map<string, FrameData>();

  // Meca500 gripper fixed joints (from meca500.xacro)
  frames.set("meca_gripper_link", {
    parent: "meca_axis_6_link",
    child: "meca_gripper_link",
    translation: { x: 0, y: 0, z: 0 },
    rotation: { x: 0, y: 0, z: 0, w: 1 },
    lastUpdate: now,
  });
  frames.set("gripper_tip_link", {
    parent: "meca_gripper_link",
    child: "gripper_tip_link",
    // origin xyz="0.04 0 -0.017" rpy="-1.5708 0 0"
    translation: { x: 0.04, y: 0, z: -0.017 },
    rotation: { x: -0.7071068, y: 0, z: 0, w: 0.7071068 },
    lastUpdate: now,
  });

  return frames;
}

export default function TfFrameViewer() {
  const [frames, setFrames] = useState<Map<string, FrameData>>(buildInitialFrames);
  const [showLabels, setShowLabels] = useState(true);
  const [showTree, setShowTree] = useState(true);
  const [showMeshes, setShowMeshes] = useState(true);

  const selectedRobotId = useRobotSelectorStore((s) => s.selectedRobotId);

  // When the selected robot changes, clear the stale TF tree so we don't
  // show ghost frames from the previous robot.
  useEffect(() => {
    setFrames(buildInitialFrames());
  }, [selectedRobotId]);

  // TF topics: try robot-namespaced first (e.g. /<robot>/tf when MoveIt2 is
  // namespaced), fall back to root /tf (single-robot / root-namespace MoveIt2).
  // Currently MoveIt2 runs at root namespace, so we always use /tf.
  // When MoveIt2 instances are namespaced per robot, switch the topic here.
  const tfTopic = "/tf";
  const tfStaticTopic = "/tf_static";

  const handleTf = useCallback((msg: TFMessage) => {
    setFrames((prev) => {
      const next = new Map(prev);
      const now = Date.now();
      for (const tf of msg.transforms) {
        next.set(tf.child_frame_id, {
          parent: tf.header.frame_id,
          child: tf.child_frame_id,
          translation: tf.transform.translation,
          rotation: tf.transform.rotation,
          lastUpdate: now,
        });
      }
      return next;
    });
  }, []);

  useTopicSubscription<TFMessage>(tfTopic, "tf2_msgs/msg/TFMessage", handleTf, 100);
  useTopicSubscription<TFMessage>(tfStaticTopic, "tf2_msgs/msg/TFMessage", handleTf, 1000);

  const frameList = useMemo(() => Array.from(frames.values()), [frames]);
  const worldPoses = useMemo(() => computeWorldPoses(frames), [frames]);

  const frameTree = useMemo(() => {
    const children = new Map<string, string[]>();
    const allChildren = new Set<string>();
    for (const f of frameList) {
      if (!children.has(f.parent)) children.set(f.parent, []);
      children.get(f.parent)!.push(f.child);
      allChildren.add(f.child);
    }
    const roots: string[] = [];
    for (const f of frameList) {
      if (!allChildren.has(f.parent) && !roots.includes(f.parent)) {
        roots.push(f.parent);
      }
    }
    return { children, roots };
  }, [frameList]);

  return (
    <div className="flex h-full">
      {/* 3D Canvas */}
      <div className="flex-1 relative">
        <Canvas camera={{ position: [1.0, 0.8, 1.0], fov: 50 }}>
          <ambientLight intensity={0.4} />
          <directionalLight position={[5, 5, 5]} intensity={0.6} />
          <Grid
            infiniteGrid
            cellSize={0.1}
            sectionSize={0.5}
            cellColor="#1e1e2e"
            sectionColor="#313244"
            fadeDistance={5}
          />
          <OrbitControls makeDefault />

          {/* ROS is Z-up, Three.js is Y-up: rotate the whole scene -90 deg around X */}
          <group rotation={[-Math.PI / 2, 0, 0]}>

          {/* Origin axes (world frame) */}
          <group>
            <Line points={[[0, 0, 0], [0.1, 0, 0]]} color="red" lineWidth={3} />
            <Line points={[[0, 0, 0], [0, 0.1, 0]]} color="green" lineWidth={3} />
            <Line points={[[0, 0, 0], [0, 0, 0.1]]} color="blue" lineWidth={3} />
            {showLabels && (
              <Text position={[0, 0.06, 0]} fontSize={0.02} color="#6b7280" anchorX="center" anchorY="bottom">
                world
              </Text>
            )}
          </group>

          {/* Meca table (collision object, not in TF — rendered at fixed pose from table.yaml) */}
          {showMeshes && (
            <group position={[0, 0.08, -0.23]} quaternion={[0.5, 0.5, 0.5, 0.5]}>
              <TableMesh url="/meshes/meca500/mecatable.stl" scale={0.001} />
            </group>
          )}

          {/* Render each frame */}
          {frameList.map((frame) => {
            const pose = worldPoses.get(frame.child);
            if (!pose) return null;
            const meshInfo = LINK_MESHES[frame.child];
            return (
              <FrameAxes
                key={frame.child}
                name={frame.child}
                position={pose.position}
                quaternion={pose.quaternion}
                showLabel={showLabels}
                meshStl={showMeshes && meshInfo ? meshInfo.stl : undefined}
                meshRpy={meshInfo?.rpy}
                meshScale={meshInfo?.scale}
              />
            );
          })}

          {/* Draw lines between parent and child in world space */}
          {frameList.map((frame) => {
            const childPose = worldPoses.get(frame.child);
            const parentPose = worldPoses.get(frame.parent);
            if (!childPose) return null;
            const pPos = parentPose ? parentPose.position : new THREE.Vector3(0, 0, 0);
            const cPos = childPose.position;
            return (
              <Line
                key={`line_${frame.child}`}
                points={[
                  [pPos.x, pPos.y, pPos.z],
                  [cPos.x, cPos.y, cPos.z],
                ]}
                color="#6b7280"
                lineWidth={1.5}
              />
            );
          })}
          </group>
        </Canvas>

        {/* Controls overlay */}
        <div className="absolute top-3 left-3 flex gap-1.5">
          <button
            onClick={() => setShowLabels(!showLabels)}
            className={clsx(
              "px-2 py-1 rounded text-[10px] flex items-center gap-1",
              showLabels ? "bg-blue-500/20 text-blue-400" : "bg-gray-800 text-gray-500"
            )}
          >
            {showLabels ? <Eye className="w-3 h-3" /> : <EyeOff className="w-3 h-3" />}
            Labels
          </button>
          <button
            onClick={() => setShowMeshes(!showMeshes)}
            className={clsx(
              "px-2 py-1 rounded text-[10px] flex items-center gap-1",
              showMeshes ? "bg-blue-500/20 text-blue-400" : "bg-gray-800 text-gray-500"
            )}
          >
            <Bone className="w-3 h-3" />
            URDF
          </button>
          <button
            onClick={() => setShowTree(!showTree)}
            className={clsx(
              "px-2 py-1 rounded text-[10px] flex items-center gap-1",
              showTree ? "bg-blue-500/20 text-blue-400" : "bg-gray-800 text-gray-500"
            )}
          >
            <List className="w-3 h-3" />
            Tree
          </button>
        </div>

        <div className="absolute bottom-3 left-3 text-[9px] text-gray-600">
          {frameList.length} frames | Drag to rotate, scroll to zoom
        </div>
      </div>

      {/* Frame tree sidebar */}
      {showTree && (
        <div className="w-56 border-l border-gray-800 overflow-y-auto overflow-x-hidden">
          <div className="px-3 py-2 border-b border-gray-800 flex items-center gap-1.5">
            <Box className="w-3.5 h-3.5 text-blue-400" />
            <span className="text-xs font-semibold">TF Frames</span>
            <span className="text-[10px] text-gray-500 ml-auto">
              {frameList.length}
            </span>
          </div>
          <div className="p-2">
            {frameTree.roots.map((root) => (
              <FrameTreeNode
                key={root}
                name={root}
                childrenMap={frameTree.children}
                frames={frames}
                worldPoses={worldPoses}
                depth={0}
              />
            ))}
            {frameTree.roots.length === 0 && (
              <p className="text-[10px] text-gray-600 text-center py-4">
                Waiting for /tf...
              </p>
            )}
          </div>
        </div>
      )}
    </div>
  );
}

/** Global cache for loaded STL geometries */
const stlCache = new Map<string, THREE.BufferGeometry>();
const stlLoader = new STLLoader();

/** Load and cache an STL mesh from the server */
function StlMesh({ url, rpy, scale }: { url: string; rpy?: [number, number, number]; scale?: number }) {
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(
    () => stlCache.get(url) ?? null
  );

  useEffect(() => {
    if (stlCache.has(url)) {
      setGeometry(stlCache.get(url)!);
      return;
    }
    stlLoader.load(
      url,
      (geo) => {
        stlCache.set(url, geo);
        setGeometry(geo);
      },
      undefined,
      (err) => console.warn("Failed to load STL:", url, err)
    );
  }, [url]);

  if (!geometry) return null;

  const euler = rpy ? new THREE.Euler(rpy[0], rpy[1], rpy[2], "XYZ") : undefined;
  const meshScale = scale ? [scale, scale, scale] as [number, number, number] : undefined;

  return (
    <mesh geometry={geometry} rotation={euler} scale={meshScale}>
      <meshStandardMaterial
        color="#8899bb"
        roughness={0.5}
        metalness={0.3}
      />
    </mesh>
  );
}

/** Static environment mesh (e.g. table) with double-sided material */
function TableMesh({ url, scale }: { url: string; scale?: number }) {
  const [geometry, setGeometry] = useState<THREE.BufferGeometry | null>(
    () => stlCache.get(url) ?? null
  );

  useEffect(() => {
    if (stlCache.has(url)) {
      setGeometry(stlCache.get(url)!);
      return;
    }
    stlLoader.load(
      url,
      (geo) => {
        geo.computeVertexNormals();
        stlCache.set(url, geo);
        setGeometry(geo);
      },
      undefined,
      (err) => console.warn("Failed to load STL:", url, err)
    );
  }, [url]);

  if (!geometry) return null;
  const meshScale = scale ? [scale, scale, scale] as [number, number, number] : undefined;

  return (
    <mesh geometry={geometry} scale={meshScale}>
      <meshStandardMaterial
        color="#556677"
        roughness={0.7}
        metalness={0.1}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

/** Render XYZ axes (and optionally a mesh) at a frame's world-space pose */
function FrameAxes({
  name,
  position,
  quaternion,
  showLabel,
  meshStl,
  meshRpy,
  meshScale,
}: {
  name: string;
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  showLabel: boolean;
  meshStl?: string;
  meshRpy?: [number, number, number];
  meshScale?: number;
}) {
  const axisLen = 0.04;
  const pos: [number, number, number] = [position.x, position.y, position.z];
  const quat: [number, number, number, number] = [quaternion.x, quaternion.y, quaternion.z, quaternion.w];

  return (
    <group position={pos} quaternion={quat}>
      {/* X axis - red */}
      <Line points={[[0, 0, 0], [axisLen, 0, 0]]} color="red" lineWidth={2} />
      {/* Y axis - green */}
      <Line points={[[0, 0, 0], [0, axisLen, 0]]} color="green" lineWidth={2} />
      {/* Z axis - blue */}
      <Line points={[[0, 0, 0], [0, 0, axisLen]]} color="blue" lineWidth={2} />
      {/* Origin sphere */}
      <mesh>
        <sphereGeometry args={[0.006, 8, 8]} />
        <meshStandardMaterial color="#a5b4fc" />
      </mesh>
      {/* STL mesh */}
      {meshStl && <StlMesh url={meshStl} rpy={meshRpy} scale={meshScale} />}
      {/* Label */}
      {showLabel && (
        <Text
          position={[0, 0.025, 0]}
          fontSize={0.012}
          color="#9ca3af"
          anchorX="center"
          anchorY="bottom"
        >
          {name}
        </Text>
      )}
    </group>
  );
}

/** Recursive frame tree sidebar node */
function FrameTreeNode({
  name,
  childrenMap,
  frames,
  worldPoses,
  depth,
}: {
  name: string;
  childrenMap: Map<string, string[]>;
  frames: Map<string, FrameData>;
  worldPoses: Map<string, WorldPose>;
  depth: number;
}) {
  const kids = childrenMap.get(name) || [];
  const frame = frames.get(name);
  const pose = worldPoses.get(name);
  const age = frame ? (Date.now() - frame.lastUpdate) / 1000 : -1;
  const stale = age > 5;

  return (
    <div style={{ paddingLeft: depth * 12 }}>
      <div className="flex items-center gap-1 py-0.5 text-[10px] hover:bg-gray-800/50 rounded px-1 group">
        <span
          className={clsx(
            "w-1.5 h-1.5 rounded-full shrink-0",
            stale && depth > 0 ? "bg-gray-600" : "bg-green-400"
          )}
        />
        <span className="text-gray-300 truncate font-mono min-w-0">{name}</span>
        {pose && (
          <span className="text-[8px] text-gray-600 ml-auto hidden group-hover:inline font-mono">
            {pose.position.x.toFixed(2)}, {pose.position.y.toFixed(2)}, {pose.position.z.toFixed(2)}
          </span>
        )}
      </div>
      {kids.map((kid) => (
        <FrameTreeNode
          key={kid}
          name={kid}
          childrenMap={childrenMap}
          frames={frames}
          worldPoses={worldPoses}
          depth={depth + 1}
        />
      ))}
    </div>
  );
}
