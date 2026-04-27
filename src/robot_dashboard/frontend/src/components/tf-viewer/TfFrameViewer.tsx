import { useState, useCallback, useMemo, useEffect, useRef } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Grid, Line, Text } from "@react-three/drei";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import { useTopicSubscription } from "../../hooks/useTopicSubscription";
import type { TFMessage } from "../../types/ros";
import { useRobotSelectorStore } from "../../stores/robot-selector-store";
import { Box, Eye, EyeOff, List, Bone } from "lucide-react";
import clsx from "clsx";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { Eyebrow } from "../ui";

interface FrameData {
  parent: string;
  child: string;
  translation: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number; w: number };
  lastUpdate: number;
}

interface WorldPose {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
}

const LINK_MESHES: Record<string, { stl: string; rpy?: [number, number, number]; scale?: number }> = {
  meca_base_link:    { stl: "/meshes/meca500/meca_500_r3_base.stl" },
  meca_axis_1_link:  { stl: "/meshes/meca500/meca_500_r3_j1.stl" },
  meca_axis_2_link:  { stl: "/meshes/meca500/meca_500_r3_j2.stl" },
  meca_axis_3_link:  { stl: "/meshes/meca500/meca_500_r3_j3.stl" },
  meca_axis_4_link:  { stl: "/meshes/meca500/meca_500_r3_j4.stl" },
  meca_axis_5_link:  { stl: "/meshes/meca500/meca_500_r3_j5.stl" },
  meca_axis_6_link:  { stl: "/meshes/meca500/meca_500_r3_j6.stl" },
  meca_gripper_link:    { stl: "/meshes/meca500/schunk_megp25e_body.stl",   rpy: [1.5708, 0, 0], scale: 0.001 },
  meca_gripper_finger1: { stl: "/meshes/meca500/schunk_megp25e_finger.stl", scale: 0.001 },
  meca_gripper_finger2: { stl: "/meshes/meca500/schunk_megp25e_finger.stl", scale: 0.001 },
};

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

function buildInitialFrames(): Map<string, FrameData> {
  const now = Date.now();
  const frames = new Map<string, FrameData>();

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
    translation: { x: 0.04, y: 0, z: -0.017 },
    rotation: { x: -0.7071068, y: 0, z: 0, w: 0.7071068 },
    lastUpdate: now,
  });

  return frames;
}

// Sociius scene colors
const SCENE_BG = "#FBF9F5"; // cream-deep
const GRID_CELL = "#EEEEEE"; // hair-soft
const GRID_SECTION = "#D7D7D7"; // hair
const TF_LINE = "#9a8f83"; // muted
const TF_LABEL = "#3a3a3a"; // ink-2
const TF_ORIGIN = "#A96D4B"; // terracotta
const MESH_COLOR = "#C9B6A0"; // warm off-white for arm meshes
const TABLE_COLOR = "#A17258"; // terracotta-soft for table

// The whole TF scene sits inside a group rotated [-π/2, 0, 0] to convert
// ROS Z-up to Three Y-up. OrbitControls operates in scene space, so click
// targets need this same rotation applied.
const ROS_TO_SCENE = new THREE.Euler(-Math.PI / 2, 0, 0);

export default function TfFrameViewer() {
  const [frames, setFrames] = useState<Map<string, FrameData>>(buildInitialFrames);
  const [showLabels, setShowLabels] = useState(true);
  const [showTree, setShowTree] = useState(true);
  const [showMeshes, setShowMeshes] = useState(true);
  const [centeredFrame, setCenteredFrame] = useState<string | null>(null);

  const orbitRef = useRef<OrbitControlsImpl | null>(null);

  const selectedRobotId = useRobotSelectorStore((s) => s.selectedRobotId);

  useEffect(() => {
    setFrames(buildInitialFrames());
  }, [selectedRobotId]);

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

  const centerOnFrame = useCallback(
    (frameName: string) => {
      // Toggle off when re-clicking the centered frame so the user has a
      // way to release tracking without picking some other frame.
      if (centeredFrame === frameName) {
        setCenteredFrame(null);
        return;
      }
      const pose = worldPoses.get(frameName);
      const controls = orbitRef.current;
      if (!pose || !controls) return;

      const sceneTarget = pose.position.clone().applyEuler(ROS_TO_SCENE);
      const oldTarget = controls.target.clone();
      const offset = controls.object.position.clone().sub(oldTarget);
      controls.target.copy(sceneTarget);
      controls.object.position.copy(sceneTarget).add(offset);
      controls.update();
      setCenteredFrame(frameName);
    },
    [centeredFrame, worldPoses],
  );

  // Live-track the centered frame: each TF update moves orbit target +
  // camera by the same delta, preserving the user's manual orbit angle
  // while the visual focus stays "stuck" to the moving frame. Skip
  // sub-millimetre deltas to avoid OrbitControls jitter at idle.
  useEffect(() => {
    if (!centeredFrame) return;
    const pose = worldPoses.get(centeredFrame);
    const controls = orbitRef.current;
    if (!pose || !controls) return;
    const newTarget = pose.position.clone().applyEuler(ROS_TO_SCENE);
    const delta = newTarget.clone().sub(controls.target);
    if (delta.lengthSq() < 1e-8) return;
    controls.target.copy(newTarget);
    controls.object.position.add(delta);
    controls.update();
  }, [worldPoses, centeredFrame]);

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
    <div className="flex h-full bg-paper">
      {/* 3D Canvas */}
      <div className="flex-1 relative" style={{ background: SCENE_BG }}>
        <Canvas camera={{ position: [1.0, 0.8, 1.0], fov: 50 }}>
          <color attach="background" args={[SCENE_BG]} />
          <ambientLight intensity={0.65} />
          <directionalLight position={[5, 5, 5]} intensity={0.5} />
          <directionalLight position={[-3, 2, -3]} intensity={0.2} />
          <Grid
            infiniteGrid
            cellSize={0.1}
            sectionSize={0.5}
            cellColor={GRID_CELL}
            sectionColor={GRID_SECTION}
            fadeDistance={5}
          />
          <OrbitControls makeDefault ref={orbitRef} />

          <group rotation={[-Math.PI / 2, 0, 0]}>
            <group>
              <Line points={[[0, 0, 0], [0.1, 0, 0]]} color="#9B2C2C" lineWidth={2.5} />
              <Line points={[[0, 0, 0], [0, 0.1, 0]]} color="#2f7d5f" lineWidth={2.5} />
              <Line points={[[0, 0, 0], [0, 0, 0.1]]} color="#51748C" lineWidth={2.5} />
              {showLabels && (
                <Text
                  position={[0, 0.06, 0]}
                  fontSize={0.022}
                  color={TF_LABEL}
                  anchorX="center"
                  anchorY="bottom"
                >
                  world
                </Text>
              )}
            </group>

            {showMeshes && (
              // Pose mirrors providers/meca500/src/meca500_bringup/config/table.yaml.
              // Quaternion [0.5, 0.5, 0.5, 0.5] == roll=π/2, pitch=0, yaw=π/2.
              // The mesh's apparent ~10cm gap to the robot base is the middle-shelf
              // clearance — closing it triggers MoveIt collision (see table.yaml).
              <group position={[0, 0.08, -0.23]} quaternion={[0.5, 0.5, 0.5, 0.5]}>
                <TableMesh url="/meshes/meca500/mecatable.stl" scale={0.001} />
              </group>
            )}

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
                  isCentered={centeredFrame === frame.child}
                  onCenter={centerOnFrame}
                />
              );
            })}

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
                  color={TF_LINE}
                  lineWidth={1}
                />
              );
            })}
          </group>
        </Canvas>

        {/* Controls overlay */}
        <div className="absolute top-3 left-3 flex gap-1.5">
          {[
            { active: showLabels, onClick: () => setShowLabels(!showLabels), icon: showLabels ? <Eye className="w-3 h-3" /> : <EyeOff className="w-3 h-3" />, label: "Labels" },
            { active: showMeshes, onClick: () => setShowMeshes(!showMeshes), icon: <Bone className="w-3 h-3" />, label: "URDF" },
            { active: showTree, onClick: () => setShowTree(!showTree), icon: <List className="w-3 h-3" />, label: "Tree" },
          ].map((b) => (
            <button
              key={b.label}
              onClick={b.onClick}
              className={clsx(
                "px-2.5 py-1 font-mono text-[10px] uppercase tracking-[0.08em] font-semibold border flex items-center gap-1.5 transition-colors",
                b.active
                  ? "bg-terracotta-tint border-terracotta text-terracotta"
                  : "bg-paper border-hair text-muted hover:text-ink-soft hover:border-ink-soft",
              )}
            >
              {b.icon}
              {b.label}
            </button>
          ))}
        </div>

        <div className="absolute bottom-3 left-3 font-mono text-[10px] text-muted tracking-[0.06em]">
          {frameList.length} frames · drag to rotate, scroll to zoom · click a frame to centre
        </div>
      </div>

      {/* Frame tree sidebar */}
      {showTree && (
        <div className="w-60 border-l border-hair overflow-y-auto overflow-x-hidden bg-paper">
          <div className="px-4 py-2.5 border-b border-hair flex items-center gap-2 bg-cream-deep">
            <Box className="w-3.5 h-3.5 text-terracotta" />
            <Eyebrow size="sm">TF Frames</Eyebrow>
            <span className="text-[10px] text-muted ml-auto font-mono tracking-[0.06em]">
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
                centeredFrame={centeredFrame}
                onCenter={centerOnFrame}
              />
            ))}
            {frameTree.roots.length === 0 && (
              <p className="font-mono text-[11px] text-muted text-center py-4 uppercase tracking-[0.08em]">
                Waiting for /tf…
              </p>
            )}
          </div>
        </div>
      )}
    </div>
  );
}

const stlCache = new Map<string, THREE.BufferGeometry>();
const stlLoader = new STLLoader();

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
      (err) => console.warn("Failed to load STL:", url, err),
    );
  }, [url]);

  if (!geometry) return null;

  const euler = rpy ? new THREE.Euler(rpy[0], rpy[1], rpy[2], "XYZ") : undefined;
  const meshScale = scale ? ([scale, scale, scale] as [number, number, number]) : undefined;

  return (
    <mesh geometry={geometry} rotation={euler} scale={meshScale}>
      <meshStandardMaterial color={MESH_COLOR} roughness={0.55} metalness={0.15} />
    </mesh>
  );
}

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
      (err) => console.warn("Failed to load STL:", url, err),
    );
  }, [url]);

  if (!geometry) return null;
  const meshScale = scale ? ([scale, scale, scale] as [number, number, number]) : undefined;

  return (
    <mesh geometry={geometry} scale={meshScale}>
      <meshStandardMaterial
        color={TABLE_COLOR}
        roughness={0.75}
        metalness={0.05}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

function FrameAxes({
  name,
  position,
  quaternion,
  showLabel,
  meshStl,
  meshRpy,
  meshScale,
  isCentered,
  onCenter,
}: {
  name: string;
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  showLabel: boolean;
  meshStl?: string;
  meshRpy?: [number, number, number];
  meshScale?: number;
  isCentered: boolean;
  onCenter: (name: string) => void;
}) {
  const axisLen = 0.04;
  const pos: [number, number, number] = [position.x, position.y, position.z];
  const quat: [number, number, number, number] = [quaternion.x, quaternion.y, quaternion.z, quaternion.w];
  const [hovered, setHovered] = useState(false);

  const handleClick = useCallback(
    (e: { stopPropagation: () => void }) => {
      e.stopPropagation();
      onCenter(name);
    },
    [name, onCenter],
  );

  const handlePointerOver = useCallback((e: { stopPropagation: () => void }) => {
    e.stopPropagation();
    setHovered(true);
    document.body.style.cursor = "pointer";
  }, []);
  const handlePointerOut = useCallback(() => {
    setHovered(false);
    document.body.style.cursor = "";
  }, []);

  const sphereRadius = isCentered || hovered ? 0.009 : 0.006;
  const sphereColor = isCentered ? "#2f7d5f" : hovered ? "#C97B4F" : TF_ORIGIN;

  return (
    <group position={pos} quaternion={quat}>
      <Line points={[[0, 0, 0], [axisLen, 0, 0]]} color="#9B2C2C" lineWidth={2} />
      <Line points={[[0, 0, 0], [0, axisLen, 0]]} color="#2f7d5f" lineWidth={2} />
      <Line points={[[0, 0, 0], [0, 0, axisLen]]} color="#51748C" lineWidth={2} />
      <mesh
        onClick={handleClick}
        onPointerOver={handlePointerOver}
        onPointerOut={handlePointerOut}
      >
        <sphereGeometry args={[sphereRadius, 12, 12]} />
        <meshStandardMaterial color={sphereColor} />
      </mesh>
      {meshStl && <StlMesh url={meshStl} rpy={meshRpy} scale={meshScale} />}
      {showLabel && (
        <Text
          position={[0, 0.025, 0]}
          fontSize={0.012}
          color={TF_LABEL}
          anchorX="center"
          anchorY="bottom"
        >
          {name}
        </Text>
      )}
    </group>
  );
}

function FrameTreeNode({
  name,
  childrenMap,
  frames,
  worldPoses,
  depth,
  centeredFrame,
  onCenter,
}: {
  name: string;
  childrenMap: Map<string, string[]>;
  frames: Map<string, FrameData>;
  worldPoses: Map<string, WorldPose>;
  depth: number;
  centeredFrame: string | null;
  onCenter: (name: string) => void;
}) {
  const kids = childrenMap.get(name) || [];
  const frame = frames.get(name);
  const pose = worldPoses.get(name);
  const age = frame ? (Date.now() - frame.lastUpdate) / 1000 : -1;
  const stale = age > 5;
  const isCentered = centeredFrame === name;

  // Each depth wraps in a div with constant marginLeft (not depth*N) so
  // nesting gives natural linear indentation, and a left border draws a
  // vertical guide line per level. Indent is small (8px) since the lines
  // make the hierarchy readable without much horizontal space — important
  // for a 6-link Meca chain in a 240px sidebar.
  return (
    <div
      className={clsx(
        depth > 0 && "ml-2 border-l border-hair-soft",
      )}
    >
      <button
        type="button"
        onClick={() => onCenter(name)}
        title="Click to centre rotation on this frame"
        className={clsx(
          "w-full flex items-center gap-1.5 py-0.5 text-[11px] pl-1.5 pr-1.5 group transition-colors text-left cursor-pointer",
          isCentered ? "bg-terracotta-tint" : "hover:bg-cream",
        )}
      >
        <span
          className={clsx(
            "w-1.5 h-1.5 rounded-full shrink-0",
            isCentered ? "bg-terracotta" : stale && depth > 0 ? "bg-muted" : "bg-ok",
          )}
        />
        <span
          className={clsx(
            "font-mono truncate min-w-0 tracking-[0.04em]",
            isCentered ? "text-terracotta" : "text-ink-soft",
          )}
        >
          {name}
        </span>
        {pose && (
          <span className="text-[9px] text-muted ml-auto hidden group-hover:inline font-mono tracking-[0.04em]">
            {pose.position.x.toFixed(2)}, {pose.position.y.toFixed(2)}, {pose.position.z.toFixed(2)}
          </span>
        )}
      </button>
      {kids.map((kid) => (
        <FrameTreeNode
          key={kid}
          name={kid}
          childrenMap={childrenMap}
          frames={frames}
          worldPoses={worldPoses}
          depth={depth + 1}
          centeredFrame={centeredFrame}
          onCenter={onCenter}
        />
      ))}
    </div>
  );
}
