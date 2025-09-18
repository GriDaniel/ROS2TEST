import { scanWorkspace } from './modules/api.js';
import { createTooltipHandlers, createPanelResizeHandlers, createFormNavigationHandlers } from './modules/ui.js';
import { renderParams, renderMarkdown } from './modules/render.js';

document.addEventListener('alpine:init', () => {

    Alpine.data('serviceManager', () => {

        const tooltip = createTooltipHandlers();
        const panelResizer = createPanelResizeHandlers();
        const formNavigator = createFormNavigationHandlers();

        return {
            //
            // Component State
            // Manages the dynamic data and UI status for the entire application.
            //
            services: [],
            dynamicData: { hosts: {} },
            activeId: null,
            nextId: 1,
            isLoading: true,
            
            // UI State
            leftWidth: 50,
            leftPanelCollapsed: false,
            rightPanelCollapsed: false,
            
            // Drag & Drop State
            draggingId: null,
            dropTargetId: null,

            // Inherited State from Modules
            ...tooltip.state,

            //
            // Initialization
            // Fetches initial workspace data and sets up the first tab.
            //
            async init() {
                this.isLoading = true;
                try {
                    this.dynamicData = await scanWorkspace();
                    this.addService();
                } catch (error) {
                    console.error("Failed to load dynamic data from server:", error);
                    alert("Error: Could not scan ROS2 workspace. See console for details.");
                } finally {
                    this.isLoading = false;
                }
            },

            //
            // Service & Tab Management
            // Core logic for adding, removing, and selecting services.
            //
            addService() {
                if (this.services.length >= 10) return;
                const id = this.nextId++;
                this.services.push({ id, host: null, package: null, key: null, view: 'inputs' });
                this.activeId = id;
            },

            removeService(id) {
                this.services = this.services.filter(s => s.id !== id);
                if (this.activeId === id && this.services.length) {
                    this.activeId = this.services.at(-1).id;
                } else if (!this.services.length) {
                    this.addService();
                }
            },

            selectHost(id, host) { this.services.find(s => s.id === id).host = host; },
            selectPackage(id, pkg) { this.services.find(s => s.id === id).package = pkg; },
            selectService(id, key) { this.services.find(s => s.id === id).key = key; },

            //
            // Event Handlers: Drag & Drop
            // Manages the user interaction for reordering tabs.
            //
            handleDragStart(event, id) {
                this.draggingId = id;
                event.dataTransfer.effectAllowed = 'move';
                event.dataTransfer.setData('text/plain', id);
            },

            handleDragOver(targetId) {
                if (targetId !== this.draggingId) {
                    this.dropTargetId = targetId;
                }
            },

            handleDrop(targetId) {
                if (this.draggingId === null || this.draggingId === targetId) {
                    this.handleDragEnd();
                    return;
                }
                const oldIndex = this.services.findIndex(s => s.id === this.draggingId);
                const newIndex = this.services.findIndex(s => s.id === targetId);
                if (oldIndex !== -1 && newIndex !== -1) {
                    const movedItem = this.services.splice(oldIndex, 1)[0];
                    this.services.splice(newIndex, 0, movedItem);
                    this.activeId = this.draggingId;
                }
                this.handleDragEnd();
            },

            handleDragEnd() {
                this.draggingId = null;
                this.dropTargetId = null;
            },

            //
            // UI Logic & Content Getters
            // Methods that compute text or handle navigation state.
            //
            handleBackClick(service) {
                if (service.view === 'help') service.view = 'inputs';
                else if (service.key) service.key = null;
                else if (service.package) service.package = null;
                else if (service.host) service.host = null;
            },

            getBackText(service) {
                if (service.view === 'help') return 'Back';
                if (service.key) return 'Service List';
                if (service.package) return 'Package List';
                if (service.host) return 'Host List';
                return '';
            },

            getSelectionTitle(service) {
                if (!service.package) {
                    return this.dynamicData.hosts[service.host]?.package_select_label || 'Select Package';
                }
                return this.dynamicData.hosts[service.host]?.service_select_label || 'Select Service';
            },

            //
            // External Modules
            // Delegates complex UI, rendering, and navigation tasks.
            //
            ...tooltip.methods,
            ...panelResizer.methods,
            ...formNavigator.methods,
            renderParams,
            renderMarkdown,
        };
    });
});