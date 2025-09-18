/**
 * @file server.js
 * @description Main application server for the ROS2 Developer Panel. This file is
 *              responsible for starting the web server, serving the frontend,
 *              and handling API requests by orchestrating the scanning process.
 * @author Gemini
 * @version 8.0.0 (Polished)
 */

const express = require('express');
const fs = require('fs').promises;
const path = require('path');
const scanner = require('./scanner'); // The powerful, reusable scanning engine

const app = express();
const port = 3000;

// --- Serve the Static Frontend ---
// All files in the 'public' directory (HTML, CSS, client-side JS) are made
// accessible to the browser.
app.use(express.static(path.join(__dirname, 'public')));

/**
 * Loads and caches configuration files, resolving internal '$ref' pointers.
 * This allows for a clean separation of configuration without re-reading files.
 * @param {string} ref - The reference string, e.g., "host_types.json#/ros2_services".
 * @param {Map<string, Object>} cache - A map to store cached file contents.
 * @returns {Promise<Object>} The resolved JSON object pointed to by the reference.
 */
const resolveRef = async (ref, cache) => {
    try {
        const [fileName, pointer] = ref.split('#');
        if (!fileName || !pointer) throw new Error(`Invalid $ref format: "${ref}".`);

        const filePath = path.join(__dirname, 'config', fileName);
        if (!cache.has(fileName)) {
            const content = await fs.readFile(filePath, 'utf8');

            cache.set(fileName, JSON.parse(content));
        }

        const pathParts = pointer.substring(1).split('/').filter(p => p);
        const result = pathParts.reduce((acc, part) => acc?.[part], cache.get(fileName));

        if (result === undefined) throw new Error(`Pointer "${pointer}" not found in "${fileName}".`);
        return result;

    } catch (error) {
        console.error(`[CONFIG ERROR] Failed to resolve ref "${ref}": ${error.message}`);
        return undefined;
    }
};

// --- API Endpoint: /scan ---
// This is the workhorse of the backend. When the frontend requests a scan,
// this function loads all configurations and delegates the analysis to the scanner.
app.get('/scan', async (req, res) => {
    console.log('\n[API] Received request to /scan. Starting analysis...');
    try {
        const refCache = new Map();
        const configPath = path.join(__dirname, 'config', 'config.json');
        const hostConfigs = JSON.parse(await fs.readFile(configPath, 'utf8'));

        const dynamicData = { hosts: {} };

        for (const config of hostConfigs) {
            const hostEntry = config.host_entry;
            if (!hostEntry?.type?.$ref) continue;

            console.log(`> Processing entry for package: "${hostEntry.package_source.label}"`);
            const hostTypeDetails = await resolveRef(hostEntry.type.$ref, refCache);

            if (!hostTypeDetails?.conventions) {
                console.warn(`  ! Skipping entry. Could not resolve valid conventions for "${hostEntry.type.$ref}".`);
                continue;
            }

            // Delegate the heavy lifting to the scanner engine
            const results = await scanner.runAnalysis(hostEntry, hostTypeDetails);
            
            // --- Data Formatting ---
            // The scanner returns a flat list of results. This section transforms
            // that list into the nested structure the frontend needs to display it.
            const hostLabel = hostTypeDetails.host_select_label;
            const packageLabel = hostEntry.package_source.label;

            if (!dynamicData.hosts[hostLabel]) {
                dynamicData.hosts[hostLabel] = {
                    label: hostLabel,
                    package_select_label: hostTypeDetails.package_select_label,
                    service_select_label: hostTypeDetails.service_select_label,
                    packages: {}
                };
            }
            if (!dynamicData.hosts[hostLabel].packages[packageLabel]) {
                dynamicData.hosts[hostLabel].packages[packageLabel] = {};
            }

            results.forEach(result => {
                dynamicData.hosts[hostLabel].packages[packageLabel][result.instanceName] = {
                    readme: `# ${result.interfaceType}\n\nDefined in: \`${result.relativePath}\``,
                    params: result.interfaceParams || []
                };
            });
        }
        
        console.log('[API] Analysis complete. Sending data to frontend.');
        res.json(dynamicData);

    } catch (error) {
        console.error('[FATAL ERROR] An unrecoverable error occurred during the scan process:', error);
        res.status(500).json({ error: 'Failed to perform scan.', message: error.message });
    }
});

// --- Start the Server ---
app.listen(port, () => {
    console.log(`✅ ROS2 Developer Panel server is running!`);
    console.log(`   Navigate to http://localhost:${port} in your browser.`);
});