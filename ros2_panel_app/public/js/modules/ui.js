/**
 * Creates state and methods for managing a global tooltip.
 * Binds methods to the Alpine component's context.
 * @returns {Object} An object containing tooltip state and methods.
 */
export function createTooltipHandlers() {
    return {
        state: {
            tooltip: { open: false, text: '', top: 0, left: 0 },
        },
        methods: {
            showTooltip(text, el) {
                if (!text || !el) return;
                const rect = el.getBoundingClientRect();
                this.tooltip = {
                    text,
                    open: true,
                    top: rect.top - 6,
                    left: rect.left + rect.width / 2,
                };
            },
            hideTooltip() {
                this.tooltip.open = false;
            },
        }
    };
}

/**
 * Creates methods for handling the resizing of vertical panels.
 * Binds methods to the Alpine component's context.
 * @returns {Object} An object containing panel resize methods.
 */
export function createPanelResizeHandlers() {
    return {
        methods: {
            startResize(e) {
                e.preventDefault();
                const container = this.$refs.container;
                document.body.classList.add('is-resizing');
                e.target.closest('.resize-handle').classList.add('is-active');

                const move = (e) => {
                    const containerRect = container.getBoundingClientRect();
                    const newWidth = ((e.clientX - containerRect.left) / containerRect.width) * 100;
                    this.leftWidth = Math.max(20, Math.min(80, newWidth));
                };

                const end = () => {
                    document.body.classList.remove('is-resizing');
                    document.querySelector('.resize-handle.is-active')?.classList.remove('is-active');
                    document.removeEventListener('mousemove', move);
                    document.removeEventListener('mouseup', end);
                };

                document.addEventListener('mousemove', move);
                document.addEventListener('mouseup', end);
            },

            resetSplit() {
                this.leftWidth = 50;
            },
        }
    };
}

/**
 * Creates methods for form input interactions and keyboard navigation.
 * Binds methods to the Alpine component's context.
 * @returns {Object} An object containing form interaction methods.
 */
export function createFormNavigationHandlers() {
    const getAllInputs = (root) => {
        return [...root.querySelectorAll('div[x-show="service.id === activeId"] input:not([type=checkbox])')];
    };
    
    return {
        methods: {
            handleInputRowClick(event) {
                if (event.target.tagName === 'INPUT') return;
                const input = event.currentTarget.querySelector('input');
                if (!input) return;
                if (input.type === 'checkbox') {
                    input.checked = !input.checked;
                } else {
                    input.focus();
                }
            },

            navigateDown(event) {
                const allInputs = getAllInputs(this.$root);
                const currentIndex = allInputs.indexOf(event.target);
                if (currentIndex > -1 && currentIndex < allInputs.length - 1) {
                    allInputs[currentIndex + 1].focus();
                }
            },

            navigateUpOnEmpty(event) {
                if (event.target.value !== '') return;
                event.preventDefault();
                const allInputs = getAllInputs(this.$root);
                const currentIndex = allInputs.indexOf(event.target);
                if (currentIndex > 0) {
                    const prevInput = allInputs[currentIndex - 1];
                    prevInput.focus();
                    prevInput.setSelectionRange(prevInput.value.length, prevInput.value.length);
                }
            },
        }
    };
}