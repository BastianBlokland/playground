#include "app/cli.h"
#include "cli/app.h"
#include "cli/parse.h"

AppType app_cli_configure(CliApp* app) {
  cli_app_register_desc(app, string_lit("Smoke demo."));
  return AppType_Console;
}

i32 app_cli_run(MAYBE_UNUSED const CliApp* app, MAYBE_UNUSED const CliInvocation* invoc) {
  return 0;
}
