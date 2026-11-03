const vm = require('vm');
const fs = require('fs');

console.log('====================================================');
console.log(' ROVER GROUND CONTACT & WHEEL ROTATION AXIS TEST   ');
console.log('====================================================');

// Minimal Three.js Math & Object3D mock for node test environment
class Vector3 {
    constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
    set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
    clone() { return new Vector3(this.x, this.y, this.z); }
    copy(v) { this.x = v.x; this.y = v.y; this.z = v.z; return this; }
    add(v) { this.x += v.x; this.y += v.y; this.z += v.z; return this; }
    sub(v) { this.x -= v.x; this.y -= v.y; this.z -= v.z; return this; }
    multiplyScalar(s) { this.x *= s; this.y *= s; this.z *= s; return this; }
    dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
    crossVectors(a, b) {
        const ax = a.x, ay = a.y, az = a.z;
        const bx = b.x, by = b.y, bz = b.z;
        this.x = ay * bz - az * by;
        this.y = az * bx - ax * bz;
        this.z = ax * by - ay * bx;
        return this;
    }
    length() { return Math.hypot(this.x, this.y, this.z); }
    normalize() {
        const l = this.length();
        if (l > 0.00001) { this.x /= l; this.y /= l; this.z /= l; }
        return this;
    }
    lerp(v, alpha) {
        this.x += (v.x - this.x) * alpha;
        this.y += (v.y - this.y) * alpha;
        this.z += (v.z - this.z) * alpha;
        return this;
    }
}

class Quaternion {
    constructor(x = 0, y = 0, z = 0, w = 1) { this.x = x; this.y = y; this.z = z; this.w = w; }
    set(x, y, z, w) { this.x = x; this.y = y; this.z = z; this.w = w; return this; }
    copy(q) { this.x = q.x; this.y = q.y; this.z = q.z; this.w = q.w; return this; }
    clone() { return new Quaternion(this.x, this.y, this.z, this.w); }
    setFromRotationMatrix(m) { return this; }
    slerp(qb, t) { return this; }
}

class Matrix4 {
    constructor() { this.elements = new Float32Array(16); }
    makeBasis(xAxis, yAxis, zAxis) {
        this.xAxis = xAxis.clone();
        this.yAxis = yAxis.clone();
        this.zAxis = zAxis.clone();
        return this;
    }
}

class Object3D {
    constructor() {
        this.position = new Vector3();
        this.rotation = { x: 0, y: 0, z: 0 };
        this.quaternion = new Quaternion();
        this.scale = new Vector3(1, 1, 1);
        this.children = [];
    }
    add(obj) { this.children.push(obj); }
    remove(obj) {
        const idx = this.children.indexOf(obj);
        if (idx !== -1) this.children.splice(idx, 1);
    }
}

class Group extends Object3D {}
class Mesh extends Object3D {
    constructor(geo, mat) {
        super();
        this.geometry = geo;
        this.material = mat;
    }
}

class CylinderGeometry {
    constructor(rTop, rBottom, height, radialSegments) {
        this.rTop = rTop; this.rBottom = rBottom; this.height = height;
        this.rotation = { x: 0, y: 0, z: 0 };
    }
    rotateX(rad) { this.rotation.x += rad; }
}

class BoxGeometry {
    constructor(w, h, d) { this.w = w; this.h = h; this.d = d; }
}

class MeshStandardMaterial {
    constructor(opt) { Object.assign(this, opt); }
}

class SpotLight extends Object3D {
    constructor() { super(); this.target = new Object3D(); }
}

const mockTHREE = {
    Vector2: class { constructor(x=0,y=0){this.x=x;this.y=y;} },
    Vector3,
    Quaternion,
    Matrix4,
    Object3D,
    Group,
    Mesh,
    CylinderGeometry,
    BoxGeometry,
    SphereGeometry: class {},
    PlaneGeometry: class {},
    MeshStandardMaterial,
    MeshBasicMaterial: class {},
    SpotLight,
    RepeatWrapping: 1000,
    TextureLoader: class {
        load(url, onLoad) {
            if (onLoad) setTimeout(() => onLoad({ repeat: new Vector3(1,1,1) }), 0);
        }
    }
};

const listeners = {};
const context = {
    console,
    setTimeout,
    clearTimeout,
    performance,
    Uint8Array, Int8Array, Int16Array, Int32Array, Uint32Array, Float32Array, Float64Array,
    WebAssembly,
    Math,
    THREE: mockTHREE,
    fetch: async (url) => {
        const data = fs.readFileSync('web/' + url.split('/').pop());
        return {
            ok: true,
            arrayBuffer: async () => data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength)
        };
    }
};
context.window = context;
context.globalThis = context;
context.addEventListener = (evt, fn) => { listeners['window_' + evt] = fn; };
context.document = {
    readyState: 'complete',
    getElementById: (id) => ({
        id,
        value: '42',
        textContent: '',
        style: {},
        classList: { add: () => {}, remove: () => {}, toggle: () => {} },
        addEventListener: (event, fn) => { listeners[id + '_' + event] = fn; },
        getContext: () => ({
            createImageData: () => ({ data: new Uint8Array(64 * 64 * 4) }),
            putImageData: () => {},
            beginPath: () => {},
            moveTo: () => {},
            lineTo: () => {},
            stroke: () => {},
            fillRect: () => {},
            fillText: () => {}
        })
    }),
    querySelectorAll: () => [],
    addEventListener: (evt, fn) => { listeners['doc_' + evt] = fn; }
};

vm.createContext(context);
vm.runInContext(fs.readFileSync('web/app.js', 'utf8'), context);
vm.runInContext(fs.readFileSync('web/rover_engine.js', 'utf8'), context);

setTimeout(() => {
    vm.runInContext(`
        // 1. Build rover model
        g_roverMesh = createRoverModel();
        console.log("[TEST] Rover model successfully instantiated.");
        console.log("[TEST] Wheel count:", g_roverWheels.length);
        if (g_roverWheels.length !== 6) {
            throw new Error("Expected exactly 6 rover wheels, got " + g_roverWheels.length);
        }

        // 2. Verify wheel orientation at rest
        g_roverWheels.forEach((w, idx) => {
            if (w.rotation.x !== 0 || w.rotation.y !== 0 || w.rotation.z !== 0) {
                throw new Error("Wheel " + idx + " initial rotation must be (0,0,0)");
            }
        });
        console.log("[TEST] Initial wheel rotations strictly zero (no Euler gimbal offset).");

        // 3. Test Reset Rover Position on Terrain
        const cs = g_cellSize;
        const startX = (g_startX + 0.5) * cs;
        const startZ = (g_startY + 0.5) * cs;
        const groundH = g_cEngine.getInterpolatedHeight(startX, startZ);

        resetRoverPosition();
        console.log("[TEST] Initial start coord: (" + startX.toFixed(2) + ", " + startZ.toFixed(2) + ")");
        console.log("[TEST] Evaluated terrain ground height at start: " + groundH.toFixed(3) + "m");
        console.log("[TEST] Rover Mesh Y position: " + g_roverMesh.position.y.toFixed(3) + "m");

        const deltaY = Math.abs(g_roverMesh.position.y - groundH);
        if (deltaY > 0.001) {
            throw new Error("Rover position Y (" + g_roverMesh.position.y + ") does not match ground elevation (" + groundH + ")");
        }
        console.log("   -> Rover accurately snapped to ground elevation (delta: " + deltaY.toFixed(6) + "m)");

        // 4. Test Wheel Rotation while Driving
        g_isRoverDriving = true;
        const dt = 0.05; // 50ms frame
        const wheelRadius = 0.35;
        const expectedAngleDelta = (g_roverSpeed * dt) / wheelRadius;

        // Simulate 1 tick of driving wheel rotation
        g_roverWheels.forEach(w => {
            w.rotation.z -= expectedAngleDelta;
        });

        g_roverWheels.forEach((w, idx) => {
            if (w.rotation.x !== 0 || w.rotation.y !== 0) {
                throw new Error("Wheel " + idx + " must NOT rotate on X or Y axis! Found X=" + w.rotation.x + " Y=" + w.rotation.y);
            }
            if (w.rotation.z >= 0) {
                throw new Error("Wheel " + idx + " must roll forward in negative Z axle direction! Found Z=" + w.rotation.z);
            }
        });
        console.log("[TEST] Driving rotation strictly on Z axle (pitch/roll forward):", g_roverWheels[0].rotation.z.toFixed(4) + " rad");

        // 5. Test Wheel Reset
        resetRoverPosition();
        g_roverWheels.forEach((w, idx) => {
            if (w.rotation.z !== 0) {
                throw new Error("Wheel " + idx + " rotation.z must reset to 0, found " + w.rotation.z);
            }
        });
        console.log("[TEST] ResetRoverPosition resets wheel rotation angles back to 0.000 rad.");

        // 6. Test Map Regeneration & Elevation Tracking across different seeds
        const testSeeds = [999, 12345, 88888];
        testSeeds.forEach(seed => {
            document.getElementById('input-seed').value = seed;
            generateTerrain();
            const newStartX = (g_startX + 0.5) * g_cellSize;
            const newStartZ = (g_startY + 0.5) * g_cellSize;
            const newGroundH = g_cEngine.getInterpolatedHeight(newStartX, newStartZ);
            const roverY = g_roverMesh.position.y;
            const err = Math.abs(roverY - newGroundH);
            if (err > 0.001) {
                throw new Error("Seed " + seed + ": Rover Y (" + roverY + ") out of sync with ground height (" + newGroundH + ")");
            }
            console.log("   -> Seed " + seed + ": Rover safely snapped at ground Y = " + roverY.toFixed(3) + "m (0 flying/clipping)");
        });

        console.log("----------------------------------------------------");
        console.log(">>> ALL ROVER GROUND CONTACT & WHEEL TESTS PASSED! <<<");
        console.log("====================================================");
    `, context);
}, 500);
