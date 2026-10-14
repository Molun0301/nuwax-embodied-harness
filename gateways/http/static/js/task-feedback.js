/* 任务栏和日志复用智能体反馈，完整结果在当前位置展开。 */
(() => {
  "use strict";
  function make(tag, text, className) {
    const node = document.createElement(tag);
    if (text != null) node.textContent = text;
    if (className) node.className = className;
    return node;
  }
  function details(initial) {
    const root = make("details", null, "task-feedback-details");
    const summary = make("summary", "反馈与完整结果");
    const fields = make("dl", null, "task-feedback-fields");
    const raw = make("pre", null, "task-feedback-data");
    const button = make("button", "读取完整结果", "btn");
    button.type = "button";
    const status = make("p", "", "meta");
    const result = make("pre", "", "task-feedback-data");
    result.hidden = true;
    let feedback = null;
    root.updateFeedback = (value) => {
      feedback = value || {};
      if (root.dataset.messageId === feedback.message_id) return;
      root.dataset.messageId = feedback.message_id || "";
      fields.replaceChildren();
      const source = feedback.source || {};
      const detail = feedback.details || {};
      for (const [label, value] of [["功能", source.display_name], ["来源", source.component],
        ["功能标识", source.capability_name], ["请求", feedback.request_id], ["任务", feedback.task_id],
        ["事件", feedback.message_id], ["失败原因", detail.failure_reason || detail.error_code], ["下一步", detail.next_action]]) {
        if (value == null || value === "") continue;
        fields.append(make("dt", label), make("dd", String(value)));
      }
      raw.textContent = JSON.stringify(feedback, null, 2);
      button.hidden = !String(feedback.result_path || "").startsWith("/api/tasks/");
    };
    button.addEventListener("click", async () => {
      const path = feedback.result_path;
      if (!String(path).startsWith("/api/tasks/")) return;
      button.disabled = true;
      status.textContent = "正在读取与智能体共用的任务结果…";
      try {
        const response = await fetch(path);
        if (!response.ok) throw new Error(`读取失败（${response.status}）`);
        const data = await response.json();
        result.textContent = JSON.stringify(data, null, 2);
        result.hidden = false;
        status.textContent = data.feedback?.message || "已读取完整结果。";
        button.textContent = "刷新完整结果";
      } catch (error) {
        status.textContent = error.message;
      } finally {
        button.disabled = false;
      }
    });
    root.append(summary, fields, raw, button, status, result);
    root.updateFeedback(initial);
    return root;
  }
  window.nuwaxFeedback = {details};
})();
