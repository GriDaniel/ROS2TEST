const fs = require('fs').promises;
const path = require('path');

// =============================================================================
// [Module] FileSystemUtils
// Provides generic helpers for navigating the file system.
// =============================================================================

const FileSystemUtils = {
    /**
     * Efficiently walks through a directory and runs a function on each file.
     */
    async processFilesRecursively(dir, action) {
        try {
            const items = await fs.readdir(dir, { withFileTypes: true });
            const promises = items.map(item => {
                const fullPath = path.join(dir, item.name);
                return item.isDirectory() ? this.processFilesRecursively(fullPath, action) : action(fullPath);
            });
            await Promise.all(promises);
        } catch (err) { /* Silently ignore unreadable directories */ }
    },
};

// =============================================================================
// [Module] InterfaceParser
// Decodes interface definition files (.srv, .msg) into a structured format.
// =============================================================================

const InterfaceParser = {
    /**
     * Reads an interface file and extracts its parameters, even handling
     * nested message types.
     */
    async parseInterfaceFile(filePath, msgMap) {
        try {
            let content = await fs.readFile(filePath, 'utf8');
            if (filePath.endsWith('.srv')) {
                content = content.split('---')[0];
            }

            const lines = content.split('\n').filter(line => line.trim() && !line.trim().startsWith('#'));

            const params = await Promise.all(lines.map(async (line) => {
                const [type, name] = line.trim().split(/\s+/);
                if (!type || !name) return null;

                const param = { type, name };
                // If a parameter is a complex type, parse its members recursively.
                if (type.includes('/')) {
                    const msgName = type.split('/')[1];
                    const msgPath = msgMap.get(msgName.toLowerCase());
                    if (msgPath) {
                        param.members = await this.parseInterfaceFile(msgPath, msgMap);
                    }
                }
                return param;
            }));
            return params.filter(Boolean); // Clean up any invalid lines
        } catch (err) {
            return [];
        }
    },
};

// =============================================================================
// [Module] ScannerCore
// The main engine that orchestrates the entire scanning and analysis process.
// =============================================================================

const ScannerCore = {
    /**
     * Catalogs all available interface files in a directory for fast lookups.
     */
    async _createInterfaceMap(interfaceDir, extensions) {
        const interfaceMap = new Map();
        await FileSystemUtils.processFilesRecursively(interfaceDir, (filePath) => {
            const ext = path.extname(filePath);
            if (extensions.includes(ext)) {
                const baseName = path.basename(filePath, ext).toLowerCase();
                interfaceMap.set(baseName, filePath);
            }
        });
        return interfaceMap;
    },

    /**
     * This is the main entry point. It runs a full analysis for one configured
     * entry, using the provided conventions to guide its logic.
     */
    async runAnalysis(entryConfig, hostTypeDetails) {
        const { conventions } = hostTypeDetails;
        const sourcePath = path.resolve(__dirname, entryConfig.package_source.path);
        const interfacePath = path.resolve(__dirname, entryConfig.interface_source.path);
        const packageName = path.basename(interfacePath);
        
        console.log(`  [1] Cataloging interfaces in "${packageName}"...`);
        const allInterfaceMap = await this._createInterfaceMap(interfacePath, conventions.interface_extensions);
        const msgMap = new Map([...allInterfaceMap].filter(([k, v]) => v.endsWith('.msg')));
        console.log(`      Found ${allInterfaceMap.size} total interface files.`);

        console.log(`  [2] Scanning source code for instantiations...`);
        const allResults = [];
        await FileSystemUtils.processFilesRecursively(sourcePath, async (filePath) => {
            if (!/\.(py|cpp|hpp|c|h|js|ts)$/.test(filePath)) return;

            try {
                const content = await fs.readFile(filePath, 'utf8');
                const instantiations = Array.from(content.matchAll(new RegExp(conventions.instantiation_regex, 'gs')));
                if (instantiations.length === 0) return;

                const relativePath = path.relative(sourcePath, filePath).replace(/\\/g, '/');
                console.log(`      Found ${instantiations.length} potential match(es) in "${relativePath}"`);

                for (const match of instantiations) {
                    const { interfaceType, instanceName } = match.groups;
                    if (!interfaceType || !instanceName) continue;
                    
                    const importPattern = conventions.import_regex_template
                        .replace('${packageName}', packageName)
                        .replace('${interfaceType}', interfaceType);

                    if (!content.match(new RegExp(importPattern, 'i'))) continue;

                    const interfaceFilePath = allInterfaceMap.get(interfaceType.toLowerCase());
                    if (!interfaceFilePath) continue;

                    const interfaceParams = await InterfaceParser.parseInterfaceFile(interfaceFilePath, msgMap);
                    allResults.push({
                        instanceName,
                        interfaceType,
                        interfaceParams,
                        relativePath,
                    });
                }
            } catch (err) { /* Ignore unreadable files */ }
        });
        
        console.log(`  [3] Analysis complete for this entry. Found ${allResults.length} valid instance(s).`);
        return allResults;
    },
};

module.exports = ScannerCore;