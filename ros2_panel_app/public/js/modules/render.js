/**
 * Renders a markdown string into HTML using the 'marked' library.
 * @param {string} markdownString The markdown content to parse.
 * @returns {string} The resulting HTML string.
 */
export function renderMarkdown(markdownString) {
    if (!markdownString || typeof marked === 'undefined') {
        return '';
    }
    return marked.parse(markdownString);
}

/**
 * Recursively renders service parameters into an HTML form structure.
 * @param {Array<Object>} params - The array of parameter objects to render.
 * @returns {string} The complete HTML string for the parameters.
 */
export function renderParams(params) {
    if (!params || !Array.isArray(params)) {
        return '';
    }

    const createInputHtml = (p) => {
        if (p.type === 'bool') {
            return `<input type="checkbox" class="form-input form-input--checkbox">`;
        }
        const inputType = p.type.includes('int') || p.type.includes('float') ? 'number' : 'text';
        const step = p.type.includes('int') ? 'step="1"' : p.type.includes('float') ? 'step="0.01"' : '';
        const numberFilterAttr = inputType === 'number' ? `onkeydown="return !['e', 'E', '+'].includes(event.key)"` : '';
        return `<input type="${inputType}" ${step} ${numberFilterAttr} class="form-input" @keydown.enter.prevent="navigateDown($event)" @keydown.backspace="navigateUpOnEmpty($event)" @keydown.delete="navigateUpOnEmpty($event)">`;
    };

    const render = (items) => items.map(p => {
        if (p.children || p.members) {
            const childParams = p.children || p.members;
            return `<details open class="param-group">
                        <summary class="param-group__summary">
                            <div class="param-group__labels">
                                <span class="param__name">${p.name}</span>
                                <span class="param__type">${p.type}</span>
                            </div>
                            <svg class="param-group__chevron" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 5l7 7-7 7" /></svg>
                        </summary>
                        <div class="param-group__children">${render(childParams)}</div>
                    </details>`;
        }
        return `<div class="param-row" @click="handleInputRowClick($event)">
                    <div class="param-row__labels">
                        <span class="param__name">${p.name}</span>
                        <span class="param__type">${p.type}</span>
                    </div>
                    ${createInputHtml(p)}
                </div>`;
    }).join('');

    return render(params);
}