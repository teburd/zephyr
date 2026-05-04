# Configuration rules for Buck2 — constraint_setting, constraint_value, platform.
# Uses built-in providers (no native.* needed): ConstraintSettingInfo,
# ConstraintValueInfo, ConfigurationInfo, PlatformInfo.
# Adapted from Buck2's nano_prelude/cfg_rules.bzl.

def _configuration_info_union(infos):
    if len(infos) == 0:
        return ConfigurationInfo(constraints = {}, values = {})
    if len(infos) == 1:
        return infos[0]
    constraints = {k: v for info in infos for (k, v) in info.constraints.items()}
    values = {k: v for info in infos for (k, v) in info.values.items()}
    return ConfigurationInfo(constraints = constraints, values = values)

def _constraint_values_to_configuration(values):
    return ConfigurationInfo(constraints = {
        info[ConstraintValueInfo].setting.label: info[ConstraintValueInfo]
        for info in values
    }, values = {})

def _constraint_setting_impl(ctx):
    return [DefaultInfo(), ConstraintSettingInfo(label = ctx.label.raw_target())]

constraint_setting = rule(
    impl = _constraint_setting_impl,
    is_configuration_rule = True,
    attrs = {},
)

def _constraint_value_impl(ctx):
    cv = ConstraintValueInfo(
        setting = ctx.attrs.constraint_setting[ConstraintSettingInfo],
        label = ctx.label.raw_target(),
    )
    return [
        DefaultInfo(),
        cv,
        ConfigurationInfo(constraints = {cv.setting.label: cv}, values = {}),
    ]

constraint_value = rule(
    impl = _constraint_value_impl,
    is_configuration_rule = True,
    attrs = {
        "constraint_setting": attrs.dep(providers = [ConstraintSettingInfo]),
    },
)

def _platform_impl(ctx):
    subinfos = (
        [dep[PlatformInfo].configuration for dep in ctx.attrs.deps] +
        [_constraint_values_to_configuration(ctx.attrs.constraint_values)]
    )
    return [
        DefaultInfo(),
        PlatformInfo(
            label = str(ctx.label.raw_target()),
            configuration = _configuration_info_union(subinfos),
        ),
    ]

platform = rule(
    impl = _platform_impl,
    is_configuration_rule = True,
    attrs = {
        "constraint_values": attrs.list(attrs.dep(providers = [ConfigurationInfo]), default = []),
        "deps": attrs.list(attrs.dep(providers = [PlatformInfo]), default = []),
    },
)
