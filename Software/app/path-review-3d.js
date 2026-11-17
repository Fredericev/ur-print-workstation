import * as THREE from "./vendor/three/three.module.min.js";
import { OrbitControls } from "./vendor/three/addons/controls/OrbitControls.js";
import { ColladaLoader } from "./vendor/three/addons/loaders/ColladaLoader.js";

const DEG = Math.PI / 180;
const DEFAULT_JOINTS = [0, -1.35, 1.55, -1.75, -1.57, 0];
const PATH_COLORS = {
  print: 0x2563d9,
  travel: 0x98a2b3,
  extrusion: 0xf79009,
  retraction: 0x7f56d9,
};

// The joint origins use the robot-specific calibration already extracted into
// workspaces/ur_ros2_ws/config/ur3e_actual_calibration.yaml. Visual origins
// match the official ur_description UR3e visual meshes bundled with the app.
const UR3E_MODEL = {
  baseFixedRpy: [0, 0, Math.PI],
  baseVisual: {
    mesh: "base",
    xyz: [0, 0, 0],
    rpy: [0, 0, Math.PI],
  },
  joints: [
    {
      name: "shoulder_pan_joint",
      xyz: [0, 0, 0.15175669655291413],
      rpy: [0, 0, -3.3891539305726243e-7],
      visual: { mesh: "shoulder", xyz: [0, 0, 0], rpy: [0, 0, Math.PI] },
    },
    {
      name: "shoulder_lift_joint",
      xyz: [-0.00006470506157420388, 0, 0],
      rpy: [1.569666958096674, 0, 0.00000699643429615883],
      visual: {
        mesh: "upperarm",
        xyz: [0, 0, 0.12],
        rpy: [Math.PI / 2, 0, -Math.PI / 2],
      },
    },
    {
      name: "elbow_joint",
      xyz: [-0.24351151612770078, 0, 0],
      rpy: [3.1379339981017003, 3.140776446145916, 3.141590226899748],
      visual: {
        mesh: "forearm",
        xyz: [0, 0, 0.027],
        rpy: [Math.PI / 2, 0, -Math.PI / 2],
      },
    },
    {
      name: "wrist_1_joint",
      xyz: [-0.21308022061379375, -0.0004228675518556208, 0.13127163719940804],
      rpy: [0.003221305822064254, 0.001264474842293318, 0.000002294291397259871],
      visual: {
        mesh: "wrist1",
        xyz: [0, 0, -0.104],
        rpy: [Math.PI / 2, 0, 0],
      },
    },
    {
      name: "wrist_2_joint",
      xyz: [0.00007433545427084375, -0.08528314119049236, 0.00002025793221502494],
      rpy: [1.5705587894393158, 0, 0.000005421801660058894],
      visual: {
        mesh: "wrist2",
        xyz: [0, 0, -0.08535],
        rpy: [0, 0, 0],
      },
    },
    {
      name: "wrist_3_joint",
      xyz: [-0.00013442575565489673, 0.09181822218034145, -0.0001463042112754946],
      rpy: [1.5692029166418646, Math.PI, -3.141592600047048],
      visual: {
        mesh: "wrist3",
        xyz: [0, 0, -0.0921],
        rpy: [Math.PI / 2, 0, 0],
      },
    },
  ],
};

function applyTransform(object, xyz, rpy) {
  object.position.set(...xyz);
  // URDF roll/pitch/yaw is a fixed-axis transform: Rz(yaw) * Ry(pitch) * Rx(roll).
  // THREE.Euler("XYZ") composes these rotations in the opposite order, which
  // pulled calibrated UR3e links apart when more than one RPY component was
  // non-zero. Compose the URDF quaternion explicitly so the browser model uses
  // the same link transforms as robot_state_publisher and MoveIt.
  const roll = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(1, 0, 0),
    Number(rpy[0]) || 0,
  );
  const pitch = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 1, 0),
    Number(rpy[1]) || 0,
  );
  const yaw = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(0, 0, 1),
    Number(rpy[2]) || 0,
  );
  object.quaternion.copy(yaw).multiply(pitch).multiply(roll);
}

function disposeObject(object) {
  object.traverse((child) => {
    child.geometry?.dispose?.();
    if (Array.isArray(child.material)) {
      child.material.forEach((material) => material.dispose?.());
    } else {
      child.material?.dispose?.();
    }
  });
}

function lineSegments(segments, kind, maxLayer) {
  const positions = [];
  const source = [];
  for (const segment of segments) {
    if (segment.kind !== kind || (maxLayer !== null && segment.layer > maxLayer)) continue;
    positions.push(
      segment.x0, segment.y0, segment.z0,
      segment.x1, segment.y1, segment.z1,
    );
    source.push(segment);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const material = new THREE.LineBasicMaterial({
    color: PATH_COLORS[kind],
    transparent: kind === "travel",
    opacity: kind === "travel" ? 0.62 : 0.98,
    depthTest: false,
  });
  const lines = new THREE.LineSegments(geometry, material);
  lines.userData.reviewSegments = source;
  lines.renderOrder = kind === "print" ? 3 : 2;
  return lines;
}

function pathEndpointPoints(segments, kind, maxLayer) {
  const positions = [];
  for (const segment of segments) {
    if (segment.kind !== kind || (maxLayer !== null && segment.layer > maxLayer)) continue;
    positions.push(segment.x1, segment.y1, segment.z1);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const points = new THREE.Points(
    geometry,
    new THREE.PointsMaterial({
      color: PATH_COLORS[kind],
      size: kind === "print" ? 0.0028 : 0.0018,
      sizeAttenuation: true,
      transparent: true,
      opacity: kind === "print" ? 0.94 : 0.46,
      depthTest: false,
    }),
  );
  points.renderOrder = kind === "print" ? 5 : 4;
  return points;
}

function eventPoints(segments, direction, maxLayer) {
  const positions = [];
  for (const segment of segments) {
    if (maxLayer !== null && segment.layer > maxLayer) continue;
    if (direction === "extrusion" && segment.extrusion_delta_mm <= 0) continue;
    if (direction === "retraction" && segment.extrusion_delta_mm >= 0) continue;
    positions.push(segment.x1, segment.y1, segment.z1);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  return new THREE.Points(
    geometry,
    new THREE.PointsMaterial({
      color: PATH_COLORS[direction],
      size: direction === "extrusion" ? 0.0045 : 0.006,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.9,
      depthTest: false,
    }),
  );
}

function createRectangleOutline(minimum, maximum, z, color, dashed = false) {
  const points = [
    new THREE.Vector3(minimum.x, minimum.y, z),
    new THREE.Vector3(maximum.x, minimum.y, z),
    new THREE.Vector3(maximum.x, maximum.y, z),
    new THREE.Vector3(minimum.x, maximum.y, z),
  ];
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const material = dashed
    ? new THREE.LineDashedMaterial({ color, dashSize: 0.012, gapSize: 0.007 })
    : new THREE.LineBasicMaterial({ color });
  const outline = new THREE.LineLoop(geometry, material);
  if (dashed) outline.computeLineDistances();
  outline.renderOrder = 1;
  return outline;
}

function createTableGrid(minimum, maximum, z) {
  const positions = [];
  const width = Number(maximum.x) - Number(minimum.x);
  const depth = Number(maximum.y) - Number(minimum.y);
  const spacing = Math.max(0.04, Math.min(0.1, Math.max(width, depth) / 12));
  for (let x = Number(minimum.x) + spacing; x < Number(maximum.x); x += spacing) {
    positions.push(x, Number(minimum.y), z, x, Number(maximum.y), z);
  }
  for (let y = Number(minimum.y) + spacing; y < Number(maximum.y); y += spacing) {
    positions.push(Number(minimum.x), y, z, Number(maximum.x), y, z);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const grid = new THREE.LineSegments(
    geometry,
    new THREE.LineBasicMaterial({
      color: 0x6f8196,
      transparent: true,
      opacity: 0.22,
      depthWrite: false,
    }),
  );
  grid.renderOrder = 1;
  return grid;
}

function createWorktable(workcellModel, bed) {
  const group = new THREE.Group();
  group.name = "print-worktable";
  const tabletop = workcellModel?.tabletop;
  const minimum = tabletop?.minimum || bed?.minimum;
  const maximum = tabletop?.maximum || bed?.maximum;
  const width = Number(maximum?.x) - Number(minimum?.x);
  const depth = Number(maximum?.y) - Number(minimum?.y);
  const planeZ = Number(tabletop?.surface_z_m ?? bed?.plane_z);
  if (!(width > 0) || !(depth > 0) || !Number.isFinite(planeZ)) return group;

  const centerX = (Number(minimum.x) + Number(maximum.x)) / 2;
  const centerY = (Number(minimum.y) + Number(maximum.y)) / 2;
  const configuredThickness = tabletop?.thickness_m;
  const thicknessKnown = configuredThickness != null
    && Number.isFinite(Number(configuredThickness))
    && Number(configuredThickness) > 0;
  const thickness = thicknessKnown
    ? Math.min(0.05, Math.max(0.001, Number(configuredThickness)))
    : null;
  const plate = new THREE.Mesh(
    thicknessKnown
      ? new THREE.BoxGeometry(width, depth, thickness)
      : new THREE.PlaneGeometry(width, depth),
    new THREE.MeshStandardMaterial({
      color: 0xc9d3df,
      roughness: 0.72,
      metalness: 0.16,
      transparent: !thicknessKnown,
      opacity: thicknessKnown ? 1 : 0.78,
      side: THREE.DoubleSide,
    }),
  );
  plate.position.set(centerX, centerY, thicknessKnown ? planeZ - thickness / 2 : planeZ);
  plate.receiveShadow = true;
  plate.castShadow = true;
  group.add(plate);
  group.add(createRectangleOutline(minimum, maximum, planeZ + 0.0008, 0x40556f));
  group.add(createTableGrid(minimum, maximum, planeZ + 0.0009));

  const configuredSupportHeight = tabletop?.support_height_m;
  const supportKnown = thicknessKnown
    && configuredSupportHeight != null
    && Number.isFinite(Number(configuredSupportHeight))
    && Number(configuredSupportHeight) > 0;
  if (supportKnown) {
    const supportHeight = Math.min(0.28, Number(configuredSupportHeight));
    const legSize = Math.min(0.05, Math.max(0.026, Math.min(width, depth) * 0.055));
    const inset = Math.max(legSize, Math.min(width, depth) * 0.07);
    const legMaterial = new THREE.MeshStandardMaterial({
      color: 0x536273,
      roughness: 0.56,
      metalness: 0.38,
    });
    const legGeometry = new THREE.BoxGeometry(legSize, legSize, supportHeight);
    const legZ = planeZ - thickness - supportHeight / 2;
    for (const x of [Number(minimum.x) + inset, Number(maximum.x) - inset]) {
      for (const y of [Number(minimum.y) + inset, Number(maximum.y) - inset]) {
        const leg = new THREE.Mesh(legGeometry, legMaterial);
        leg.position.set(x, y, legZ);
        leg.castShadow = true;
        group.add(leg);
      }
    }
  }
  group.userData.visualOnly = tabletop?.visual_only === true;
  group.userData.surfaceZ = planeZ;
  return group;
}

function createMeasuredBedOverlay(bed, bedCheck) {
  const group = new THREE.Group();
  group.name = "measured-print-bed-overlay";
  const width = Number(bed?.maximum?.x) - Number(bed?.minimum?.x);
  const depth = Number(bed?.maximum?.y) - Number(bed?.minimum?.y);
  const planeZ = Number(bed?.plane_z);
  if (!(width > 0) || !(depth > 0) || !Number.isFinite(planeZ)) return group;

  const safeMinimum = bedCheck?.usable_minimum;
  const safeMaximum = bedCheck?.usable_maximum;
  if (
    safeMinimum
    && safeMaximum
    && Number(safeMinimum.x) < Number(safeMaximum.x)
    && Number(safeMinimum.y) < Number(safeMaximum.y)
  ) {
    const safeWidth = Number(safeMaximum.x) - Number(safeMinimum.x);
    const safeDepth = Number(safeMaximum.y) - Number(safeMinimum.y);
    const safeArea = new THREE.Mesh(
      new THREE.PlaneGeometry(safeWidth, safeDepth),
      new THREE.MeshBasicMaterial({
        color: 0x17a673,
        transparent: true,
        opacity: 0.09,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    safeArea.position.set(
      (Number(safeMinimum.x) + Number(safeMaximum.x)) / 2,
      (Number(safeMinimum.y) + Number(safeMaximum.y)) / 2,
      planeZ + 0.0007,
    );
    safeArea.renderOrder = 0;
    group.add(safeArea);
    group.add(createRectangleOutline(
      safeMinimum,
      safeMaximum,
      planeZ + 0.001,
      0x17a673,
      true,
    ));
  }
  return group;
}

function createMeasuredTool(tcp) {
  const group = new THREE.Group();
  group.name = "measured-print-tool";
  const offset = new THREE.Vector3(
    Number(tcp?.offset?.x) || 0,
    Number(tcp?.offset?.y) || 0,
    Number(tcp?.offset?.z) || 0,
  );
  if (offset.length() <= 0.0001) return group;
  const tip = new THREE.Mesh(
    new THREE.SphereGeometry(0.003, 18, 12),
    new THREE.MeshStandardMaterial({ color: 0xf79009, metalness: 0.35, roughness: 0.4 }),
  );
  tip.position.copy(offset);
  tip.castShadow = true;
  group.add(tip);
  return group;
}

function fitCamera(camera, controls, objectOrBounds, direction = "iso") {
  const box = objectOrBounds?.isBox3
    ? objectOrBounds
    : new THREE.Box3().setFromObject(objectOrBounds);
  if (box.isEmpty()) return;
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const radius = Math.max(sphere.radius, 0.25);
  const directions = {
    // contentRoot is ROS Z-up and rosRoot converts it to Three.js Y-up.
    // These vectors are therefore expressed in the displayed Y-up frame so
    // the default isometric camera looks down onto the tabletop, not at its
    // underside.
    iso: new THREE.Vector3(1.2, 1.0, 1.35),
    top: new THREE.Vector3(0.001, 1, 0.001),
    front: new THREE.Vector3(1, 0.25, 0.001),
    side: new THREE.Vector3(0.001, 0.25, 1),
  };
  const vector = (directions[direction] || directions.iso).normalize();
  camera.up.set(0, 1, 0);
  if (direction === "top") camera.up.set(0, 0, -1);
  controls.target.copy(sphere.center);
  camera.position.copy(sphere.center).addScaledVector(vector, radius * 2.6);
  camera.near = Math.max(0.001, radius / 200);
  camera.far = radius * 50;
  camera.updateProjectionMatrix();
  controls.update();
}

function interpolateTrajectory(result, progress) {
  const trajectory = result?.trajectory;
  if (!trajectory?.length) return null;
  if (trajectory.length === 1) return trajectory[0].positions;
  const clamped = Math.min(1, Math.max(0, progress));
  let upperIndex = trajectory.findIndex((point) => Number(point.progress) >= clamped);
  if (upperIndex < 0) upperIndex = trajectory.length - 1;
  if (upperIndex === 0) return trajectory[0].positions;
  const lower = trajectory[upperIndex - 1];
  const upper = trajectory[upperIndex];
  const span = Math.max(1e-9, Number(upper.progress) - Number(lower.progress));
  const ratio = (clamped - Number(lower.progress)) / span;
  return lower.positions.map(
    (value, index) => Number(value) + (Number(upper.positions[index]) - Number(value)) * ratio,
  );
}

class PathReviewViewer {
  constructor(container, review, options = {}) {
    this.container = container;
    this.review = review;
    this.options = options;
    this.maxLayer = null;
    this.progress = 0;
    this.moveitResult = null;
    this.jointNodes = [];
    this.pathObjects = [];
    this.destroyed = false;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf8fafc);
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.001, 30);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.domElement.setAttribute("aria-label", "Interactive 3D UR robot path");
    this.renderer.domElement.tabIndex = 0;
    container.replaceChildren(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = true;
    this.controls.minDistance = 0.08;
    this.controls.maxDistance = 8;

    this.rosRoot = new THREE.Group();
    this.rosRoot.rotation.x = -Math.PI / 2;
    this.scene.add(this.rosRoot);
    this.contentRoot = new THREE.Group();
    this.rosRoot.add(this.contentRoot);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xc9d0da, 1.8));
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.4);
    keyLight.position.set(1.6, 2.4, 2.8);
    keyLight.castShadow = true;
    this.scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0xdce8ff, 0.8);
    fillLight.position.set(-2, 1, -1);
    this.scene.add(fillLight);

    this.worktable = createWorktable(review.workcell_model, review.print_bed_m);
    this.contentRoot.add(this.worktable);
    this.bed = createMeasuredBedOverlay(review.print_bed_m, review.print_bed_check);
    this.contentRoot.add(this.bed);
    this.cursor = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 18, 12),
      new THREE.MeshStandardMaterial({
        color: 0x12a15b,
        emissive: 0x075d36,
        emissiveIntensity: 0.35,
      }),
    );
    this.cursor.castShadow = true;
    this.contentRoot.add(this.cursor);

    this.buildPath();
    this.buildRobot().then(() => {
      if (!this.destroyed) {
        requestAnimationFrame(() => this.fitCameraPreset("iso"));
      }
    });
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.renderer.domElement.addEventListener("click", (event) => this.pickSegment(event));
    this.resize();
    this.setProgress(0);
    requestAnimationFrame(() => this.fitCameraPreset("iso"));
    this.animate();
  }

  buildPath() {
    this.pathObjects.forEach((object) => {
      this.contentRoot.remove(object);
      disposeObject(object);
    });
    this.pathObjects = [
      lineSegments(this.review.path.segments, "travel", this.maxLayer),
      lineSegments(this.review.path.segments, "print", this.maxLayer),
      pathEndpointPoints(this.review.path.segments, "travel", this.maxLayer),
      pathEndpointPoints(this.review.path.segments, "print", this.maxLayer),
      eventPoints(this.review.path.segments, "extrusion", this.maxLayer),
      eventPoints(this.review.path.segments, "retraction", this.maxLayer),
    ];
    this.pathObjects.forEach((object) => this.contentRoot.add(object));
  }

  async buildRobot() {
    const loader = new ColladaLoader();
    const robotRoot = new THREE.Group();
    applyTransform(robotRoot, [0, 0, 0], UR3E_MODEL.baseFixedRpy);
    this.contentRoot.add(robotRoot);
    this.robotRoot = robotRoot;

    const addMesh = async (visual, parent) => {
      try {
        const collada = await loader.loadAsync(
          `/assets/ur3e-visual/${visual.mesh}.dae`,
        );
        const model = collada.scene;
        model.traverse((child) => {
          if (!child.isMesh) return;
          child.castShadow = true;
          child.receiveShadow = true;
          const materials = Array.isArray(child.material)
            ? child.material
            : [child.material];
          materials.filter(Boolean).forEach((material) => {
            if ("shininess" in material) material.shininess = 38;
            material.needsUpdate = true;
          });
        });
        applyTransform(model, visual.xyz, visual.rpy);
        parent.add(model);
      } catch (error) {
        this.options.onModelError?.(error);
      }
    };

    await addMesh(UR3E_MODEL.baseVisual, robotRoot);
    let parent = robotRoot;
    for (let index = 0; index < UR3E_MODEL.joints.length; index += 1) {
      const joint = UR3E_MODEL.joints[index];
      const originNode = new THREE.Group();
      applyTransform(originNode, joint.xyz, joint.rpy);
      const rotationNode = new THREE.Group();
      originNode.add(rotationNode);
      parent.add(originNode);
      this.jointNodes.push(rotationNode);
      await addMesh(joint.visual, rotationNode);
      parent = rotationNode;
    }
    const flangeNode = new THREE.Group();
    applyTransform(flangeNode, [0, 0, 0], [0, -Math.PI / 2, -Math.PI / 2]);
    parent.add(flangeNode);
    const tool0Node = new THREE.Group();
    applyTransform(tool0Node, [0, 0, 0], [Math.PI / 2, 0, Math.PI / 2]);
    flangeNode.add(tool0Node);
    try {
      const response = await fetch("/api/workcell/height");
      if (!response.ok) throw new Error("Printhead geometry is unavailable.");
      const heightSetup = await response.json();
      const configuration = heightSetup.nozzle;
      const visualPath = configuration.visual_mesh_web_path;
      if (typeof visualPath !== "string" || !visualPath.startsWith("/assets/printhead/") ||
          !visualPath.endsWith(".dae") || visualPath.includes("..")) {
        throw new Error("The installed printhead visual mesh is unavailable.");
      }
      // The web mesh and CAD transform come from the same installed geometry
      // profile as the collision review, so an old asset cannot silently stand
      // in for a revised head. A source revision also refreshes browser caches.
      const revision = configuration.source_sha256 || configuration.geometry_revision || "";
      const mounted = await loader.loadAsync(`${visualPath}?revision=${encodeURIComponent(revision)}`);
      const model = mounted.scene;
      model.traverse((child) => {
        if (child.isMesh) {
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });
      const origin = configuration.tool0_to_cad_origin_m;
      const rotation = configuration.tool0_to_cad_rpy_deg;
      applyTransform(model, [origin.x, origin.y, origin.z],
        [rotation.roll * DEG, rotation.pitch * DEG, rotation.yaw * DEG]);
      tool0Node.add(model);
      tool0Node.add(createMeasuredTool({ offset: heightSetup.installed_tcp_m }));
    } catch (error) {
      this.options.onModelError?.(error);
    }
    this.setRobotJoints(DEFAULT_JOINTS);
    this.setProgress(this.progress);
  }

  focusBounds() {
    const box = new THREE.Box3();
    const focusObjects = [
      this.bed,
      this.worktable,
      this.cursor,
      this.robotRoot,
      ...this.pathObjects,
    ].filter(Boolean);
    focusObjects.forEach((object) => box.expandByObject(object));
    return box;
  }

  fitCameraPreset(preset) {
    fitCamera(this.camera, this.controls, this.focusBounds(), preset);
  }

  setRobotJoints(positions) {
    if (!positions || positions.length !== 6) return;
    this.jointNodes.forEach((joint, index) => {
      joint.rotation.z = Number(positions[index]) || 0;
    });
  }

  setMoveItResult(result) {
    this.moveitResult = result;
    const first = result?.trajectory?.[0]?.positions;
    if (Array.isArray(first) && first.length === 6) {
      this.setRobotJoints(first);
    } else {
      this.setRobotJoints(DEFAULT_JOINTS);
    }
    this.setProgress(this.progress);
  }

  setLayer(maxLayer) {
    this.maxLayer = Number.isFinite(Number(maxLayer)) ? Number(maxLayer) : null;
    this.buildPath();
    this.setProgress(this.progress);
  }

  setProgress(progress) {
    this.progress = Math.min(1, Math.max(0, Number(progress) || 0));
    const visible = this.review.path.segments.filter(
      (segment) => this.maxLayer === null || segment.layer <= this.maxLayer,
    );
    if (visible.length) {
      const scaled = this.progress * Math.max(0, visible.length - 1);
      const index = Math.min(visible.length - 1, Math.floor(scaled));
      const segment = visible[index];
      const local = Math.min(1, Math.max(0, scaled - index));
      this.cursor.position.set(
        segment.x0 + (segment.x1 - segment.x0) * local,
        segment.y0 + (segment.y1 - segment.y0) * local,
        segment.z0 + (segment.z1 - segment.z0) * local,
      );
      this.options.onProgress?.({
        progress: this.progress,
        segment,
        index,
        count: visible.length,
      });
    }
    const joints = interpolateTrajectory(this.moveitResult, this.progress);
    if (joints) this.setRobotJoints(joints);
  }

  setCameraPreset(preset) {
    this.fitCameraPreset(preset);
  }

  pickSegment(event) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const point = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.params.Line.threshold = 0.008;
    raycaster.setFromCamera(point, this.camera);
    const candidates = this.pathObjects.filter((object) => object.isLineSegments);
    const intersection = raycaster.intersectObjects(candidates, false)[0];
    if (!intersection) return;
    const source = intersection.object.userData.reviewSegments || [];
    const segmentIndex = Math.floor((intersection.index || 0) / 2);
    const segment = source[segmentIndex];
    if (!segment) return;
    const allSegments = this.review.path.segments;
    const index = allSegments.indexOf(segment);
    if (index >= 0) {
      this.setProgress(index / Math.max(1, allSegments.length - 1));
      this.options.onSelectSegment?.(segment);
    }
  }

  resize() {
    const rect = this.container.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    this.renderer.setSize(rect.width, rect.height, false);
    this.camera.aspect = rect.width / rect.height;
    this.camera.updateProjectionMatrix();
  }

  animate() {
    if (this.destroyed) return;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.animationFrame = requestAnimationFrame(() => this.animate());
  }

  destroy() {
    this.destroyed = true;
    cancelAnimationFrame(this.animationFrame);
    this.resizeObserver?.disconnect();
    this.controls.dispose();
    disposeObject(this.scene);
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}

window.URPathReview3D = {
  mount(container, review, options) {
    return new PathReviewViewer(container, review, options);
  },
};

window.dispatchEvent(new CustomEvent("ur-path-review-3d-ready"));
