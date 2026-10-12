"""操作任务复用公共能力、任务管理和状态流，不拥有模型推理。"""
from functools import partial

from contracts.capabilities import CapabilityAvailability, CapabilityDescriptor, CapabilityExecutionMode, CapabilityRiskLevel
from contracts.skills import SkillCategory, SkillDescriptor
from core.errors import GatewayError
from services.manipulation.service import ManipulationService

NAME = 'execute_manipulation_task'
DESCRIPTION = ('设备已配置不代表后端当前可用；目录不探测远端，执行受理前实时验证能力与运行会话。'
               '把自然语言操作指令交给已配置设备的动作模型，并跟踪真实物理结果。'
               '当前G1示例面向苹果入盘任务；不保证任意物品或跨房间搬运。'
               '受理不代表完成，取消等待同任务的设备暂停证明。')


def service(runtime):
    existing = getattr(runtime, 'manipulation_service', None)
    if existing is None:
        from core.settings import ManipulationConfig
        existing = ManipulationService(getattr(runtime.config, 'manipulation', ManipulationConfig()))
        runtime.manipulation_service = existing
    return existing


def availability(runtime):
    supported, reason = service(runtime).availability()
    return CapabilityAvailability(name=NAME, supported=supported, reason=reason,
        metadata={'availability_scope': 'local_configuration', 'backend_verification': 'not_checked',
                  'verification_point': 'bind'})


def execute(runtime, arguments, *, requested_by=None, request_context=None):
    implementation = service(runtime)
    try:
        parameters = implementation.parameters(arguments)
    except ValueError as error:
        raise GatewayError(str(error), error_code='invalid_params', http_status=422) from error
    # 请求受理时绑定代次，后台等待资源/调度不能把旧指令发送到新机器人会话。
    binding = implementation.bind()
    descriptor = runtime.capability_registry.get_descriptor(NAME)
    return runtime.task_manager.submit(NAME, lambda context: implementation.run(context, binding, parameters),
        input_payload=parameters, requested_by=requested_by,
        required_resources=list(descriptor.required_resources), timeout_sec=parameters['timeout_sec'], deferred_interrupt=True,
        metadata=runtime._build_task_request_metadata(capability_name=NAME,
            requested_by=requested_by, request_context=request_context))


def register_capability(runtime):
    service(runtime)
    descriptor = CapabilityDescriptor(name=NAME, display_name='执行物体操作任务', description=DESCRIPTION,
        execution_mode=CapabilityExecutionMode.ASYNC_TASK, risk_level=CapabilityRiskLevel.HIGH,
        input_schema={'type':'object', 'properties': {
            'instruction': {'type':'string','minLength':1,'maxLength':2000},
            'timeout_sec': {'type':'number','minimum':1,'maximum':1800,'default':120},
            'max_steps': {'type':'integer','minimum':1,'maximum':10000,'default':1440}},
            'required':['instruction'], 'additionalProperties':False}, output_schema={'type':'object'},
        required_resources=['base_motion','navigation'], timeout_sec=120, cancel_supported=True,
        exposed_to_agent=True, tags=['manipulation','device_task'])
    runtime.capability_registry.register(descriptor, partial(execute, runtime), owner='gateway_runtime',
        metadata={'capability_layer':'platform'}, overwrite=True)
    return (NAME,)


def register_skill(runtime):
    runtime.skill_registry.register(SkillDescriptor(name=NAME, display_name='执行物体操作任务', description=DESCRIPTION,
        category=SkillCategory.TASK, capability_name=NAME, exposed_to_agent=True), overwrite=True)
