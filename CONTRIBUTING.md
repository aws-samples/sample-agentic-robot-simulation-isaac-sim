# Contributing Guidelines

Thank you for your interest in contributing to our project. Whether it's a bug report, new feature, correction, or additional
documentation, we greatly value feedback and contributions from our community.

Please read through this document before submitting any issues or pull requests to ensure we have all the necessary
information to effectively respond to your bug report or contribution.


## Reporting Bugs/Feature Requests

We welcome you to use the GitHub issue tracker to report bugs or suggest features.

When filing an issue, please check existing open, or recently closed, issues to make sure somebody else hasn't already
reported the issue. Please try to include as much information as you can. Details like these are incredibly useful:

* A reproducible test case or series of steps
* The version of our code being used
* Any modifications you've made relevant to the bug
* Anything unusual about your environment or deployment


## Contributing via Pull Requests
Contributions via pull requests are much appreciated. Before sending us a pull request, please ensure that:

1. You are working against the latest source on the *main* branch.
2. You check existing open, and recently merged, pull requests to make sure someone else hasn't addressed the problem already.
3. You open an issue to discuss any significant work - we would hate for your time to be wasted.

To send us a pull request, please:

1. Fork the repository.
2. Modify the source; please focus on the specific change you are contributing. If you also reformat all the code, it will be hard for us to focus on your change.
3. Ensure local tests pass (see "Running the checks locally" below).
4. Commit to your fork using clear commit messages.
5. Send us a pull request, answering any default questions in the pull request interface.
6. Pay attention to any automated CI failures reported in the pull request, and stay involved in the conversation.

GitHub provides additional document on [forking a repository](https://help.github.com/articles/fork-a-repo/) and
[creating a pull request](https://help.github.com/articles/creating-a-pull-request/).

### Running the checks locally

These are the same checks CI runs (`.github/workflows/ci.yml`):

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python --require-hashes --no-deps \
  --only-binary :all: -r tests/requirements-ci.lock    # Linux x86_64 and macOS arm64
.venv/bin/python -m unittest discover -s tests
git ls-files -z '*.sh' | xargs -0 shellcheck -x --severity=warning
python3 scripts/check_citations.py --allow-missing-revisions docs/security/THREAT_MODEL.md
```

### Changes that need a second reviewer

Some files decide whether the simulated robot may enter the tank, when it must retreat, and
what runs as root on the host. A pull request that touches any of them needs an approving
review from a maintainer other than its author, and must say which threat-model entries
(`docs/security/THREAT_MODEL.md`) it affects:

* gas limits, test order, entry and retreat logic: `agent/mission.py`, `agent/spot_agent.py`,
  `agent/prompts.py`, `sim/gas_field.py`
* anything that runs on the host or with operator credentials: `host/`, `scripts/`, `tools/`,
  the Dockerfiles and the dependency lock files
* the policies, account settings and infrastructure an adopter applies: `docs/iam/` and
  `infra/`, and the audit trail's attribution and chain head: `agent/operator_console.py`

This sample supports the Isaac Sim simulator only. Pull requests that add a path to command a
physical robot will not be accepted: a real robot needs its own safety case and security
review (see "Roadmap to the real robot" in the README).


## Finding contributions to work on
Looking at the existing issues is a great way to find something to contribute on. As our projects, by default, use the default GitHub issue labels (enhancement/bug/duplicate/help wanted/invalid/question/wontfix), looking at any 'help wanted' issues is a great place to start.


## Code of Conduct
This project has adopted the [Amazon Open Source Code of Conduct](https://aws.github.io/code-of-conduct).
For more information see the [Code of Conduct FAQ](https://aws.github.io/code-of-conduct-faq) or contact
opensource-codeofconduct@amazon.com with any additional questions or comments.


## Security issue notifications
If you discover a potential security issue in this project we ask that you notify AWS/Amazon Security via our [vulnerability reporting page](http://aws.amazon.com/security/vulnerability-reporting/). Please do **not** create a public github issue.


## Licensing

See the [LICENSE](LICENSE) file for our project's licensing. We will ask you to confirm the licensing of your contribution.
