# main_project/create_project.py
import os

# All multiline strings are "raw" (r"""...""") to prevent escape sequence warnings.
file_structure = {
    "project_web/config.json": r"""
{
  "host_type": "ros2 services",
  "source_folder": "some_other_dir/ros2_scanner_system",
  "interface_source_folder": "some_other_dir/ros2_scanner_system/example_interfaces"
}
""",
    "project_web/scanner.js": r"""
// project_web/scanner.js

/**
 * A clean, modular, and verbose JavaScript system for finding and analyzing
 * ROS2 command instantiations and their associated interfaces.
 *
 * This version contains corrected, robust recursive search logic for interfaces
 * and enhanced logging for easier debugging.
 *
 * @author Gemini
 * @version 6.2.0 (Corrected & Log-Enhanced)
 */

const fs = require('fs');
const path = require('path');

/**
 * [Module] Configuration and Mapping
 */
const ConventionMapper = {
  getDetailsForHostType: (hostType) => {
    const conventionMap = {
      'ros2 services': {
        import_notation_regex: /from ([\w.]+) import ([\w\s,]+)/,
        instantiation_analysis_regex: /\.create_service\(\s*([\w\d_]+)\s*,\s*['"]([^'"]+)['"]/,
      },
    };
    return conventionMap[hostType] || null;
  },
};

/**
 * [Module] Interface Analyzer
 */
const InterfaceAnalyzer = {
  /**
   * [CORRECTED] Recursively searches a directory for a specific file.
   * Includes logging to trace the search path.
   * @param {string} directory - The directory to start searching in.
   * @param {string} fileName - The name of the file to find (e.g., "AddTwoInts.srv").
   * @returns {string|null} The full path to the file, or null if not found.
   */
  _findSrvFileRecursive: (directory, fileName) => {
    // console.log(`    ... Searching in: ${directory}`); // Uncomment for extreme verbosity
    try {
      const items = fs.readdirSync(directory, { withFileTypes: true });
      for (const item of items) {
        const fullPath = path.join(directory, item.name);
        if (item.isDirectory()) {
          const result = InterfaceAnalyzer._findSrvFileRecursive(fullPath, fileName);
          if (result) return result;
        } else if (item.isFile() && item.name === fileName) {
          return fullPath;
        }
      }
    } catch (err) {
      console.warn(`    [WARN] Could not read directory: ${directory}`);
    }
    return null;
  },
  
  parseSrvContent: (content) => {
    const params = [];
    const requestPart = content.split('---')[0];
    const lines = requestPart.split('\n');
    for (const line of lines) {
      const trimmedLine = line.trim();
      if (trimmedLine && !trimmedLine.startsWith('#')) {
        const parts = trimmedLine.split(/\s+/);
        if (parts.length >= 2) params.push({ type: parts[0], name: parts[1] });
      }
    }
    return params;
  },

  analyzeServiceInterfaces: (service, interfaceSourcePath) => {
    const fileContent = fs.readFileSync(service.file, 'utf8');
    const conventions = ConventionMapper.getDetailsForHostType('ros2 services');
    const importRegex = new RegExp(conventions.import_notation_regex, 'g');
    
    let match;
    while ((match = importRegex.exec(fileContent)) !== null) {
      const importedItems = match[2].split(',').map(item => item.trim());

      if (importedItems.includes(service.interfaceType)) {
        console.log(`    -> Found matching import for "'${service.interfaceType}'". Searching for .srv file...`);
        const srvFileName = `${service.interfaceType}.srv`;
        const srvFilePath = InterfaceAnalyzer._findSrvFileRecursive(interfaceSourcePath, srvFileName);
        
        if (srvFilePath) {
          console.log(`    -> Success! Parsing "'${srvFilePath}'"`);
          const srvContent = fs.readFileSync(srvFilePath, 'utf8');
          service.interfaceParams = InterfaceAnalyzer.parseSrvContent(srvContent);
          return service;
        }
      }
    }
    console.warn(`    [WARN] Could not find a matching .srv file for interface "'${service.interfaceType}'" anywhere within "'${interfaceSourcePath}'"`);
    return null;
  },
};

/**
 * [Module] File System Scanner
 */
const FileScanner = {
  runFullAnalysis: (config) => {
    const projectRoot = path.resolve(__dirname, '..');
    const sourcePath = path.join(projectRoot, config.source_folder);
    const interfaceSourcePath = path.join(projectRoot, config.interface_source_folder);
    const conventions = ConventionMapper.getDetailsForHostType(config.host_type);

    if (!conventions) throw new Error(`Host type "'${config.host_type}'" is not supported.`);
    if (!fs.existsSync(sourcePath)) throw new Error(`Source folder does not exist: ${sourcePath}`);
    if (!fs.existsSync(interfaceSourcePath)) throw new Error(`Interface source folder does not exist: ${interfaceSourcePath}`);

    console.log(`Scanning source folder: ${sourcePath}`);
    console.log(`Using interface source folder: ${interfaceSourcePath}`);
    
    console.log(`\n[PASS 1] Scanning for "'${config.host_type}'" instantiations...`);
    const potentialServices = [];
    const walk = (currentDir) => {
      const items = fs.readdirSync(currentDir, { withFileTypes: true });
      for (const item of items) {
        const fullPath = path.join(currentDir, item.name);
        if (item.isDirectory()) {
          walk(fullPath);
        } else if (item.isFile()) {
          try {
            const content = fs.readFileSync(fullPath, 'utf8');
            const instantiationRegex = new RegExp(conventions.instantiation_analysis_regex, 'g');
            let match;
            while ((match = instantiationRegex.exec(content)) !== null) {
              potentialServices.push({
                file: fullPath,
                relativePath: path.relative(sourcePath, fullPath),
                interfaceType: match[1],
                serviceName: match[2],
              });
            }
          } catch (err) { /* Ignore */ }
        }
      }
    };
    walk(sourcePath);
    console.log(`  -> Found ${potentialServices.length} potential service instantiation(s).`);

    console.log(`\n[PASS 2] Analyzing services to link interfaces...`);
    const finalResults = [];
    for (const service of potentialServices) {
      console.log(`  -> Analyzing service in "'${service.relativePath}'"`);
      const result = InterfaceAnalyzer.analyzeServiceInterfaces(service, interfaceSourcePath);
      if (result) {
        finalResults.push(result);
      }
    }
    return finalResults;
  },
};

/**
 * [Main Execution Block]
 */
const main = () => {
  console.log('--- ROS2 Smart Scanner Initialized ---');
  try {
    const config = JSON.parse(fs.readFileSync(path.join(__dirname, 'config.json'), 'utf8'));
    const results = FileScanner.runFullAnalysis(config);
    results.sort((a, b) => a.relativePath.localeCompare(b.relativePath));
    console.log('\n--- Analysis Complete ---');
    if (results.length > 0) {
      console.log('Found the following services and their interface parameters:');
      results.forEach(result => {
        const normalizedPath = result.relativePath.replace(/\\/g, '/');
        console.log(`\n[FILE]: ${normalizedPath}`);
        console.log(`  ├─ Service Name: '${result.serviceName}'`);
        console.log(`  └─ Uses Interface: ${result.interfaceType}`);
        if (result.interfaceParams && result.interfaceParams.length > 0) {
          result.interfaceParams.forEach(param => {
            console.log(`     ├─ Param: ${param.name} (type: ${param.type})`);
          });
        } else {
          console.log(`     └─ (No request parameters)`);
        }
      });
    } else {
      console.log('No services with parseable interface imports were found.');
    }
  } catch (error) {
    console.error('\n[ERROR] An error occurred:', error.message);
  } finally {
    console.log('\n----------------------------------------------------');
  }
};

main();
""",
    # --- The nested ros2_scanner_system structure ---
    "some_other_dir/ros2_scanner_system/src/services/service_node.py": r"""
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        return response
""",
    "some_other_dir/ros2_scanner_system/example_interfaces/srv/AddTwoInts.srv": r"""
# Request
int64 a
int64 b

---

# Response
int64 sum
""",
}

def create_project():
    """Generates the project structure from the file_structure dictionary."""
    print("Generating JS project with corrected recursive interface search...")
    base_dir = os.getcwd()
    for path, content in file_structure.items():
        full_path = os.path.join(base_dir, path)
        dir_name = os.path.dirname(full_path)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        
        with open(full_path, 'w', encoding='utf-8') as f:
            f.write(content.strip())
    
    print("Project generation complete!")
    print("\nTo run the system:")
    print("1. cd main_project/project_web")
    print("2. node scanner.js")

if __name__ == "__main__":
    create_project()