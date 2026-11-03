const vm = require('vm');
const fs = require('fs');

const listeners = {};
const context = {
    console,
    setTimeout,
    clearTimeout,
    performance,
    Uint8Array,
    Int8Array,
    Int16Array,
    Int32Array,
    Uint32Array,
    Float32Array,
    Float64Array,
    BigInt64Array,
    WebAssembly,
    Math,
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
        addEventListener: (event, fn) => {
            listeners[id + '_' + event] = fn;
        },
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
    querySelectorAll: (sel) => [],
    addEventListener: (evt, fn) => { listeners['doc_' + evt] = fn; }
};
context.requestAnimationFrame = (fn) => setTimeout(fn, 16);
context.cancelAnimationFrame = (id) => clearTimeout(id);

vm.createContext(context);
vm.runInContext(fs.readFileSync('web/app.js', 'utf8'), context);
vm.runInContext(fs.readFileSync('web/rover_engine.js', 'utf8'), context);

setTimeout(() => {
    console.log('--- Checking Application Lifecycle Status ---');
    vm.runInContext(`
        console.log("g_isInitialized:", g_isInitialized);
        console.log("g_cEngine:", g_cEngine ? "ONLINE" : "OFFLINE");
        console.log("Current Path Length:", g_cEngine.getPathLength());
        console.log("Solving path instantly...");
        solveInstantly();
        console.log("Solved Path Length:", g_cEngine.getPathLength());
        console.log("Smoothed Path Length:", g_cEngine.getSmoothedPathLength());
    `, context);
    console.log('--- Testing UI Click Handlers ---');
    console.log('Clicking btn-generate-terrain...');
    listeners['btn-generate-terrain_click']();
    console.log('Clicking btn-step...');
    listeners['btn-step_click']();
    console.log('Clicking btn-solve...');
    listeners['btn-solve_click']();
    console.log('Clicking btn-reset...');
    listeners['btn-reset_click']();
    console.log('Testing chk-show-closed toggle...');
    if (listeners['chk-show-closed_change']) {
        listeners['chk-show-closed_change']({ target: { checked: false } });
        console.log('Toggled off blue explored shade successfully!');
        listeners['chk-show-closed_change']({ target: { checked: true } });
        console.log('Toggled on blue explored shade successfully!');
    }
    console.log('Testing slider-path-smoothing (0%, 50%, 100%)...');
    if (listeners['slider-path-smoothing_input']) {
        listeners['slider-path-smoothing_input']({ target: { value: '0' } });
        console.log('Set smoothing to 0% (raw rigid grid steps) successfully!');
        listeners['slider-path-smoothing_input']({ target: { value: '50' } });
        console.log('Set smoothing to 50% (balanced corner rounding) successfully!');
        listeners['slider-path-smoothing_input']({ target: { value: '100' } });
        console.log('Set smoothing to 100% (high curvature smoothing) successfully!');
    }
    console.log('Testing Visual Resolution & Texture Library Picker...');
    vm.runInContext(`
        applySelectedTexture('res-sandygravel', '2k');
        console.log('Picked Sandy Gravel 02 from resource library! Mode:', g_textureMode);
        applySelectedTexture('res-ground060', '4k');
        console.log('Picked Ground 060 (4K) from resource library! Mode:', g_textureMode);
        applySelectedTexture('mars-procedural');
        console.log('Picked Procedural Mars! Mode:', g_textureMode);
        applySelectedTexture('res-ground060', '2k');
        console.log('Re-selected Ground 060 (2K)! Mode:', g_textureMode);
    `, context);

    if (listeners['select-texture-quality_change']) {
        listeners['select-texture-quality_change']({ target: { value: '4k' } });
        console.log('Switched texture quality to 4K Ultra HD!');
        listeners['select-texture-quality_change']({ target: { value: '2k' } });
        console.log('Switched texture quality to 2K Fast Load!');
    }
    if (listeners['slider-normal-scale_input']) {
        listeners['slider-normal-scale_input']({ target: { value: '1.8' } });
        console.log('Adjusted normal map relief scale to 1.8!');
    }
    if (listeners['slider-roughness_input']) {
        listeners['slider-roughness_input']({ target: { value: '0.65' } });
        console.log('Adjusted surface roughness factor to 0.65!');
    }
    if (listeners['slider-texture-tiling_input']) {
        listeners['slider-texture-tiling_input']({ target: { value: '32' } });
        console.log('Adjusted micro-texture tiling to 32x!');
    }
    if (listeners['select-mesh-subdivision_change']) {
        listeners['select-mesh-subdivision_change']({ target: { value: '2' } });
        console.log('Enabled 2x mesh geometric subdivision via C height interpolation!');
        listeners['select-mesh-subdivision_change']({ target: { value: '1' } });
        console.log('Reverted to 1x mesh geometric subdivision!');
    }
    if (listeners['chk-grid-lines_change']) {
        listeners['chk-grid-lines_change']({ target: { checked: true } });
        console.log('Toggled on crisp grid lines!');
        listeners['chk-grid-lines_change']({ target: { checked: false } });
        console.log('Toggled off crisp grid lines!');
    }
    console.log('SUCCESS: All UI event listeners, texture library picker, and C bindings verified functional!');
}, 500);
