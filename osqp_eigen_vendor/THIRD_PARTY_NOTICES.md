# Third-Party Notices

This package vendors source code from the following third-party project:

## osqp-eigen

- Upstream project: `robotology/osqp-eigen`
- Upstream repository: <https://github.com/robotology/osqp-eigen>
- Vendored version: `v0.10.3`
- Vendored source path: `osqp_eigen_vendor/third_party/osqp-eigen`
- License: `BSD-3-Clause`

The upstream `LICENSE` file is preserved in the vendored source tree and is
also installed with this package as `share/osqp_eigen_vendor/osqp-eigen-LICENSE`.

This vendor package itself contains ROBOTIS packaging glue under the
`Apache-2.0` license.

## Runtime Dependency

`osqp-eigen` depends on OSQP at build and runtime. In this workspace, OSQP is
provided by the external ROS package `osqp_vendor`, which is licensed under
`Apache-2.0`.
