# 專案層 F´ 設定覆寫（照 lib/fprime docs/user-manual/build-system/configuration.md「Overriding Configuration in a Project」）。
# 只放有改的檔；檔名必須跟 F´ default/config 一樣。
register_fprime_config(
        RocketPlan01Config
    CONFIGURATION_OVERRIDES
        "${CMAKE_CURRENT_LIST_DIR}/CommandDispatcherImplCfg.hpp"
    INTERFACE
)
