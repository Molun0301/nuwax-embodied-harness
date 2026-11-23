/* 接口调用：能力经与智能体相同的 /api/capabilities/<名>/invoke；管理服务经 /api/manager/*。
   失败时抛出带后端原因的错误（error.code 为后端错误码）。 */

/* init 可带 signal（需要超时的调用自行中止）。 */
export async function invoke(name, args = {}, init = {}) {
  const response = await fetch('/api/capabilities/' + encodeURIComponent(name) + '/invoke', { method: 'POST',
    headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ arguments: args }), ...init });
  const value = await response.json();
  if (!response.ok) {
    const error = new Error(value.error?.message || '服务返回 ' + response.status);
    error.code = value.error?.code || '';
    error.status = response.status;
    throw error;
  }
  return value;
}

// 能力目录只取一次，供各页面共用；失败后下次调用重新读取。
let catalog = null;
export function capabilities() {
  if (!catalog) catalog = invoke('list_capabilities').then(value => value.capabilities)
    .catch(error => { catalog = null; throw error; });
  return catalog;
}

export async function json(url, init = {}) {
  const response = await fetch(url, { cache: 'no-store', ...init });
  const value = await response.json();
  if (!response.ok) {
    const error = new Error(value.error?.message || value.message || '服务返回 ' + response.status);
    error.code = value.error?.code || '';
    error.status = response.status;
    throw error;
  }
  return value;
}

/* 管理服务：写请求先读当前会话的校验值；未登录时提示到设备管理页登录（沿用原控制台软件更新的做法）。 */
export async function manager(path, { method = 'GET', body, timeout = 15000 } = {}) {
  const headers = {};
  if (method !== 'GET') {
    headers['Content-Type'] = 'application/json';
    headers['X-NEH-CSRF'] = (await manager('session')).csrf_token;
  }
  const response = await fetch('/api/manager/' + path, {
    method, headers, body: body === undefined ? undefined : JSON.stringify(body),
    signal: AbortSignal.timeout(timeout), cache: 'no-store',
  });
  const result = await response.json();
  if (response.status === 401) throw new Error('请先在设备管理页登录');
  if (!response.ok) {
    const details = Array.isArray(result.error?.details) ? '：' + result.error.details.join('；') : '';
    throw new Error((result.error?.message || '设备管理程序请求失败') + details);
  }
  return result;
}
