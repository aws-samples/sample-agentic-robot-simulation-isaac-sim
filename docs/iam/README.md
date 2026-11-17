# IAM and host baseline for the sim host

The sim host is built by hand (`infra/` holds templates for supporting resources only, such
as the session log group below), so this page is the baseline to build it to. It replaces
broad managed policies with three small policy documents. Each passed IAM Access Analyzer
`validate-policy` with no findings, the instance role's session-log statements (threat
model T38) included, and so did the key and topic policies of the `infra/` templates. Validate
them all again after you edit them:

```bash
aws accessanalyzer validate-policy --policy-type IDENTITY_POLICY \
  --policy-document file://docs/iam/instance-role-policy.json
```

Replace `<ACCOUNT_ID>` with your account ID, and `<SESSION_LOG_KEY_ID>` with the key ID
that the session-logging stack outputs ([below](#session-manager-one-os-account-per-operator-and-session-logs-off-the-host)).
The documents assume `us-west-2` and the
`us.anthropic.claude-sonnet-5` inference profile (the agent's default model,
`agent/spot_agent.py`). If you change either, change every ARN that names it, and check in
the Amazon Bedrock documentation which Regions the profile routes requests to.

| File | Attach to | Grants |
|---|---|---|
| [`instance-role-trust-policy.json`](instance-role-trust-policy.json) | The sim host's instance role (trust policy) | EC2 may assume the role. |
| [`instance-role-policy.json`](instance-role-policy.json) | The same role, next to the AWS managed policy `AmazonSSMManagedInstanceCore` | Bedrock invoke on the one inference profile, and on its foundation model only through that profile; `bedrock:ApplyGuardrail` on the one guardrail; read of the NICE DCV licence bucket; for the SSM Agent, writing session logs to the streams of the one session log group and using its KMS key only through CloudWatch Logs. **No Secrets Manager access**: nothing on the host reads a secret. |
| [`presenter-policy.json`](presenter-policy.json) | The people who present the demo | SSM port forwarding through the `SpotDemo-PortForward` document only, to instances tagged `spot-demo=sim-host`, and managing their own sessions. No SSM shell, no Run Command, no secrets. |
| [`port-forwarding-document.json`](port-forwarding-document.json) | An SSM document you create once, named `SpotDemo-PortForward` | The same as `AWS-StartPortForwardingSession`, except that it forwards only 8443 (DCV) and 8080 (HUD), to the same local port. |
| [`session-preferences.json`](session-preferences.json) | The account's Session Manager preferences: the Session document `SSM-SessionManagerRunShell` | Shells run as each operator's own OS account (Run As, no default user), streamed to the encrypted log group of [`infra/session-logging.yaml`](../../infra/session-logging.yaml), 20 minute idle timeout. |
| [`operator-runas-guardrail.json`](operator-runas-guardrail.json) | Every principal that can open a shell on the host (operators and administrators), and anyone who can change their tags | Grants nothing. Denies setting or removing the `SSMSessionRunAs` tag on any IAM user or role, and passing it as a session tag, so nobody it is attached to can make Session Manager start their shell as another operator's account. |

**The guardrail is on by default**, so the policy's `ApplyTheOneGuardrail` statement is
part of the shipped file. Replace `<GUARDRAIL_ID>` with the `GuardrailId` output of
[`infra/bedrock-guardrail.yaml`](../../infra/bedrock-guardrail.yaml), as you replace
`<ACCOUNT_ID>` and `<SESSION_LOG_KEY_ID>`. The agent refuses to start with no guardrail
configured unless the operator passes `--no-guardrail`, which warns and is recorded in
the session record; that flag needs no permission, so removing the statement does not
disable the guardrail, it only makes a guardrail session fail at the first model call.

## Roles

Keep two kinds of human access apart (threat model T29):

* **Presenters** use `presenter-policy.json`. It is enough for `scripts/connect-dcv.sh` and
  `scripts/connect-hud.sh` with `SPOT_PORT_FORWARD_DOCUMENT=SpotDemo-PortForward`, and
  nothing else. Create the document once:

  ```bash
  aws ssm create-document --name SpotDemo-PortForward --document-type Session \
    --document-format JSON --content file://docs/iam/port-forwarding-document.json
  ```

  **The DCV desktop is root-equivalent** (threat model T1, T28): DCV logs in as `ubuntu`,
  which is in the `docker` group and has `sudo`. So a presenter who has the DCV password
  can act as root on the host, whatever this policy says. Give the password only to people
  you would give a root shell, rotate it before each event, and use the HUD tunnel alone
  for anyone else.
* **Administrators** start and stop the instance, open shells, deploy
  (`scripts/deploy-to-host.sh`), fetch logs (`scripts/fetch-log.sh`) and read the DCV
  password secret. Give them a separate role with short-lived credentials (IAM Identity
  Center or an assumed role), not long-lived access keys. An SSM shell or Run Command is
  root on the host, so treat this role as host administrator.

The DCV login password is the one secret. Create it in Secrets Manager (for example
`spot-demo/dcv-password`; `scripts/connect-dcv.sh` reads the name from
`SPOT_DCV_SECRET_ID`), and grant `secretsmanager:GetSecretValue` on it to the
administrator role only. Rotate it before each event. Pulling the Isaac Sim image needs no
NVIDIA NGC key.

## Host settings to apply

| Setting | Value | Why (threat model) |
|---|---|---|
| Security group | No inbound rules. Never open 8443 or 8080. | T1, T2, T25, T44 |
| Public IP | None (use a subnet with a NAT gateway or VPC endpoints for egress) | T1, T27 |
| Instance metadata | IMDSv2 required, hop limit 1 | T26 |
| Listeners off loopback | None but the SSM agent, DCV (on loopback once `host/configure_dcv.sh apply` has run) and the DHCP client: turn off SSH (`ssh.socket`, `ssh.service`) and, once a desktop is installed, `avahi-daemon` | T44 |
| Shutdown behaviour | `stop` (the EC2 default): `host/idle_stop.sh` powers the host off, which must stop it, not terminate it | T39 |
| Tag | `spot-demo=sim-host` (the presenter policy keys on it) | T29 |
| EBS volume | Encrypted | T37 |
| Session Manager preferences | [`session-preferences.json`](session-preferences.json): Run As each operator's own OS account (the `SSMSessionRunAs` tag, no default user), session data streamed to the encrypted log group; see [below](#session-manager-one-os-account-per-operator-and-session-logs-off-the-host) | T29, T38 |

```bash
aws ec2 modify-instance-metadata-options --instance-id <instance-id> \
  --http-tokens required --http-put-response-hop-limit 1
```

With `--network=host`, every container on the host can reach the instance metadata
service, so any of them could take the role's credentials (T26). The policy above bounds
what those credentials can do to invoking one model, reading the DCV licence and adding
events to the session log group (which also lets them list its streams), plus what
the AWS managed policy `AmazonSSMManagedInstanceCore` grants, which includes reading any
Systems Manager Parameter Store parameter in the account and Region (`ssm:GetParameter`
and `ssm:GetParameters` on `*`). Keep secrets out of Parameter Store in this account, or
replace the managed policy with a narrower copy. To narrow access further,
`host/imds_guard.sh apply` adds firewall rules so that only root and the agent
container's user can reach the metadata service; the launch scripts refuse to start a
container until those rules are in place, after every boot.

## Session Manager: one OS account per operator, and session logs off the host

Threat model T38. With the shipped Session Manager defaults every shell on the host is the
shared `ssm-user`, so the mission JSONL names a shared account, and the JSONL exists only on
the host, where `ubuntu` and root can delete or rewrite it. These steps make the login that
the agent records the operator's own, and put a copy of the log's chain head where host
users cannot rewrite it:

* **Run As.** Session Manager starts each shell as the OS account named by the
  `SSMSessionRunAs` tag on the operator's IAM role or user. There is no default user, so a
  principal without the tag gets no session at all; Session Manager never falls back to
  `ssm-user`, and it does not support `root` as a Run As account. The operator then runs
  `sudo -iu ubuntu`, sudo keeps the operator's account in `SUDO_USER`,
  `host/run_agent.sh` passes it to the agent, and the mission record names it. The agent
  and `run_mission.py` refuse to start on a shared or unknown login (`ssm-user`, `ubuntu`,
  `root`, none) unless `--unattributed-run` records the run as unattributed.
* **Session logs.** Every shell session is streamed, as it happens, to the CloudWatch Logs
  log group `/spot-demo/session-manager`, encrypted with its own KMS key. The operator
  console prints the mission log's chain head (`[audit head]` lines: record count and the
  SHA-256 of the last record) at session start, at mission start, after each confirmation,
  at mission end and at session end, so those lines leave the host as they are printed. The
  instance role can add log events but cannot change or delete them; a host user who takes
  its credentials could add a forged line, or a whole stream named like an operator's
  session, but not remove the real ones. That is why a check takes the session's ID from
  Session Manager's own history, not from a stream's name, and uses every head line of that
  session, not only the last. `docs/RUNBOOK.md` ("Check a
  log against the session log") says how. Session Manager needs the `screen` utility on
  Linux to log a session in full; Ubuntu Server has it by default.

**These preferences are account- and Region-wide**: every Session Manager shell in the
account and Region gets them, including other instances' and other teams'. Use a dedicated
account for the demo. They do not cover port forwarding (the presenters'
`SpotDemo-PortForward` sessions start no shell, and Session Manager does not log port
forwarding or SSH sessions), Run Command (`scripts/deploy-to-host.sh`, `scripts/fetch-log.sh`
run as root and are recorded by CloudTrail and in the SSM command history only), or the DCV desktop's terminal (DCV logs
in as `ubuntu`, so the agent there has no invoking login and refuses to start without
`--unattributed-run`).

An administrator applies them once, in this order (a session started while the log group
is missing or unencrypted fails: Session Manager checks the group when
`cloudWatchEncryptionEnabled` is true):

1. **Deploy the log group and key** from [`infra/session-logging.yaml`](../../infra/session-logging.yaml)
   (parameters: `LogGroupName`, default `/spot-demo/session-manager`, which the two
   documents here also name; `RetentionInDays`, default 365, one year or more):

   ```bash
   aws cloudformation deploy --region us-west-2 --stack-name spot-demo-session-logging \
     --template-file infra/session-logging.yaml
   aws cloudformation describe-stacks --region us-west-2 --stack-name spot-demo-session-logging \
     --query 'Stacks[0].Outputs'      # KeyId replaces <SESSION_LOG_KEY_ID>
   ```

2. **Update the instance role's policy** with [`instance-role-policy.json`](instance-role-policy.json)
   (the session-log statements: `logs:CreateLogStream` and `logs:PutLogEvents` on the
   group's streams, `logs:DescribeLogStreams` on the group, `logs:DescribeLogGroups`, which
   supports no resource-level permission, and `kms:GenerateDataKey`, `kms:Decrypt` and
   `kms:DescribeKey` on the one key, only through CloudWatch Logs), and validate it with IAM
   Access Analyzer as above.
3. **Tag each operator's IAM role or user** with their OS account name, one account per
   person. With roles from IAM Identity Center or another identity provider, pass
   `SSMSessionRunAs` as a session tag instead; Session Manager reads the session tag first,
   then the role's tag.

   ```bash
   aws iam tag-role --role-name <operator-role> --tags Key=SSMSessionRunAs,Value=<os-account>
   aws iam tag-user --user-name <operator-user> --tags Key=SSMSessionRunAs,Value=<os-account>
   ```

   The operators' policy needs `ssm:StartSession` on the default Session document
   `SSM-SessionManagerRunShell` as well as on the instance. Administrators are operators
   too: without the tag they get no shell either. The tag decides whose name the record
   carries, so a principal that can set it on itself, or pass it as a session tag, can run as
   another operator: attach [`operator-runas-guardrail.json`](operator-runas-guardrail.json)
   to every operator and administrator principal, and set the tags from an identity that it
   does not apply to (a separate break-glass role, or your identity provider's attribute
   mapping).
4. **Create each OS account on the host**, from an existing shell (Run Command, or a session
   opened before step 5): `sudo bash /opt/spot-demo/host/add_operator.sh <os-account>`.
   It creates the account with no password and a sudoers rule that allows only
   `sudo -iu ubuntu` (`status <os-account>` checks it, `remove <os-account>` undoes it).
5. **Apply the preferences.** Keep the current ones first, so you can go back, outside the
   checkout (they name the account's log group, bucket and key, T36):

   ```bash
   aws ssm get-document --region us-west-2 --name SSM-SessionManagerRunShell \
     --document-version '$LATEST' --query Content --output text > ~/spot-demo-previous-preferences.json
   aws ssm update-document --region us-west-2 --name SSM-SessionManagerRunShell \
     --content file://docs/iam/session-preferences.json --document-version '$LATEST'
   ```

   If `get-document` says the document does not exist (preferences were never saved in this
   account and Region), create it instead:
   `aws ssm create-document --region us-west-2 --name SSM-SessionManagerRunShell --document-type Session --document-format JSON --content file://docs/iam/session-preferences.json`.
6. **Check it**: `aws ssm get-document --region us-west-2 --name SSM-SessionManagerRunShell --document-version '$LATEST'`
   shows the new content; a new session's `whoami` prints your OS account; the log group has
   a stream named after your session ID, with your commands in it.

**To roll back**, put the saved preferences back with the same `update-document` command
and `--content file://$HOME/spot-demo-previous-preferences.json` (if you created the document in step 5,
update it with a copy of `session-preferences.json` whose `runAsEnabled` is `false` and
whose `cloudWatchLogGroupName` is empty: shells are then `ssm-user` again, and not logged).
Then remove the host accounts (`sudo bash /opt/spot-demo/host/add_operator.sh remove
<os-account>`), the `SSMSessionRunAs` tags (`aws iam untag-role --role-name <operator-role>
--tag-keys SSMSessionRunAs`) and the session-log statements of the instance-role policy.
Deleting the stack (`aws cloudformation delete-stack --stack-name spot-demo-session-logging`)
keeps the log group and the key, because they hold the audit trail; delete them yourself
once you no longer need the logs (`aws logs delete-log-group --log-group-name
/spot-demo/session-manager`, then `aws kms schedule-key-deletion --key-id <key-id>`).

## What this sample does not set up for you

* **Budgets, idle stop and recovery** (T39): shipped as optional steps that you review
  and turn on, not applied for you. `infra/cost-guardrails.yaml` creates an AWS Budgets
  alert on the account's cost, a CloudWatch alarm that stops the instance (EC2 stop
  action) after a sustained low-CPU period, and status-check alarms
  ([`infra/README.md`](../../infra/README.md)); `host/idle_stop.sh apply` powers the
  host off after 60 idle minutes ([RUNBOOK](../RUNBOOK.md), step 0). Neither needs a
  change to the instance role. Stop the instance after every session anyway, and see
  "Cleanup" in the [README](../../README.md).
* **Data perimeter**: conditions such as `aws:EC2InstanceSourceVPC` can stop the role's
  credentials being used from outside your VPC. They depend on how your VPC reaches AWS
  (VPC endpoints or NAT), so they are left to you; see the IAM documentation on
  [data perimeters](https://docs.aws.amazon.com/IAM/latest/UserGuide/access_policies_data-perimeters.html).
