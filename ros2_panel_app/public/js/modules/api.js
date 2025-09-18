/**
 * Fetches the ROS2 workspace structure from the '/scan' endpoint.
 * Throws an error if the network response is not OK.
 * @returns {Promise<Object>} A promise that resolves to the JSON data of the workspace.
 */
export async function scanWorkspace() {
    const response = await fetch('/scan');

    if (!response.ok) {
        throw new Error(`Server scan failed with status: ${response.statusText}`);
    }

    return await response.json();
}