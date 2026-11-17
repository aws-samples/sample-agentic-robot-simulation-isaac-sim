# Sample infrastructure (`infra/`)

The CloudFormation templates here are sample infrastructure for the sim host.
[`bedrock-guardrail.yaml`](bedrock-guardrail.yaml) is **required**, because the agent is
guardrail-on by default and refuses to start without one; the other two are optional and
the demo runs without them. Each one is a starting point for an adopter, who
reviews it, adjusts it to their account's rules and deploys it into their own account:
nothing in this repository deploys them, and they were checked with `cfn-lint` and
`checkov` only, not deployed for this review (so the guardrail's policies have not been
exercised against a real prompt; its effect on the demo's own hazard language is the
reason the violence and misconduct filters are set low, and a rehearsal is the way to
confirm the strengths suit your wording). They do not create the host, its
instance role or its network; those are built by hand from the baseline in
[`docs/iam/README.md`](../docs/iam/README.md).

Check a template before you deploy it, from the repository root, with the tool
versions it was checked with (pinned, so a new release does not run unreviewed on a
machine that holds AWS credentials; move a pin on purpose, after reading the release):

```bash
uvx cfn-lint@1.59.0 infra/*.yaml                                # no output: no finding
uvx checkov@3.4.0 -d infra --framework cloudformation            # Passed checks: 7, Failed checks: 0
```

## Bedrock guardrail (`bedrock-guardrail.yaml`)

The agent applies an Amazon Bedrock guardrail to the operator's latest instruction and
the model's reply, and it is on by default: with neither `SPOT_GUARDRAIL_ID` /
`SPOT_GUARDRAIL_VERSION` nor `--guardrail-id` / `--guardrail-version` the agent refuses
to start, unless the operator passes `--no-guardrail`, which warns and is recorded in
the session record (threat model T6, T10, T11, M11). Deploy this stack once per account
and Region before the first session.

A guardrail is **probabilistic and is not a safety function**. What stops the robot is
deterministic and in the code: the facade's command limits, the typed allow-listed robot
data, the simulator-only backend check and the operator's `y/N` confirmations. Those hold
whether or not this stack is deployed. The guardrail is the second layer in front of them,
and it leaves a CloudWatch metric and a trace when it fires.

**What it creates**

| Resource | What it does |
|---|---|
| `AgentGuardrail` (Bedrock) | Four policies. **Content filters**: `PROMPT_ATTACK` at HIGH on the input (`OutputStrength` must be `NONE`; it is an input-side filter), `HATE`, `INSULTS` and `SEXUAL` at HIGH both ways, and `VIOLENCE` and `MISCONDUCT` at **LOW on purpose** — a confined-space demo's ordinary language is about atmospheres that kill, injury, rescue and emergency retreat, and a higher strength blocks the demo's own subject matter. **Denied topic** `RealRobotOperation`: commanding real hardware, or getting past the simulation-only restriction. **Word policy**: the managed profanity list, nothing custom. **Sensitive information**: `NAME`, `ADDRESS`, `EMAIL`, `PHONE` and `USERNAME` anonymised, `PASSWORD`, `AWS_ACCESS_KEY` and `AWS_SECRET_KEY` blocked — the sample is built to hold no personal data (permits take role labels, not names; `--attendant` enforces it), so this is the backstop for an operator who types one anyway. |
| `AgentGuardrailVersion` (Bedrock) | An immutable version. The agent is always given a version, never `DRAFT`, so a session is screened by the policy that was reviewed. |
| `InterventionAlarm` (CloudWatch) | An alarm on `AWS/Bedrock/Guardrails` `InvocationsIntervened` for this guardrail and version, so an intervention is noticed and not only recorded. Missing data is treated as not breaching, because no intervention publishes no data point. **Always created.** With `AlarmTopicArn` it notifies that topic; without one it has no actions, which is still an alarm that changes state, shows in the console and the API, and annotates the dashboard. |
| `GuardrailDashboard` (CloudWatch) | **Always created**, because a dashboard needs no SNS topic, no e-mail address and no KMS key, so there is no reason to leave a guardrail unobserved. Four widgets: interventions against total calls (annotated with the alarm), interventions split by policy type (content filters, the denied topic, the word policy, sensitive information), the latency the guardrail adds (average and p99), and a note on how to read an intervention and on what a guardrail is not. The first three dashboards in an account, up to 50 metrics each, are in the CloudWatch free tier; this one uses 8 metric lines. |

Bedrock bills guardrail evaluation by text unit (Amazon Bedrock pricing); a demo session
is a handful of short instructions and replies. The alarm is one standard CloudWatch alarm;
the dashboard is free unless the account already has three.

**Parameters**

| Parameter | Default | Meaning |
|---|---|---|
| `GuardrailName` | `spot-demo-agent` | The guardrail's name; must match `^[0-9a-zA-Z-_]+$`. |
| `AlarmTopicArn` | (empty) | Optional. A topic for the intervention alarm, for example the `AlarmTopicArn` output of [`cost-guardrails.yaml`](cost-guardrails.yaml). Empty does **not** mean no alarm: the alarm is still created, it just has no actions. |
| `InterventionThreshold` | 1 | Interventions in one 5-minute period before the alarm fires. 1 reports every one. |

**Deploy** it from the Mac, as an administrator, in the host's Region:

```bash
aws cloudformation deploy --region us-west-2 --stack-name spot-demo-bedrock-guardrail \
  --template-file infra/bedrock-guardrail.yaml
aws cloudformation describe-stacks --region us-west-2 \
  --stack-name spot-demo-bedrock-guardrail --query 'Stacks[0].Outputs'
```

Five outputs. `GuardrailId` and `GuardrailVersion` are the two variables the host sets for
the agent; `DashboardName` and `InterventionAlarmName` are where to watch interventions;
and `GuardrailArn` replaces `<GUARDRAIL_ID>` in the `ApplyTheOneGuardrail` statement of
[`docs/iam/instance-role-policy.json`](../docs/iam/instance-role-policy.json). Set the
variables on the host so every session gets them (`host/run_agent.sh` passes them into
the container), then update the instance role:

```bash
# on the host, as an administrator, once per host
sudo tee -a /etc/environment >/dev/null <<'EOF'
SPOT_GUARDRAIL_ID=<GuardrailId output>
SPOT_GUARDRAIL_VERSION=<GuardrailVersion output>
EOF
```

**Check** it:

```bash
aws bedrock get-guardrail --region us-west-2 --guardrail-identifier <GuardrailId> \
  --guardrail-version <GuardrailVersion> \
  --query '[name,status,contentPolicy.filters[].[type,inputStrength,outputStrength]]'
aws cloudwatch describe-alarms --region us-west-2 \
  --alarm-names <InterventionAlarmName output> \
  --query 'MetricAlarms[].[AlarmName,StateValue,AlarmActions]'
aws cloudwatch get-dashboard --region us-west-2 --dashboard-name <DashboardName output> \
  --query 'DashboardName'
```

The alarm starts in `INSUFFICIENT_DATA` and stays there until the guardrail is called,
which is correct: no intervention publishes no data point. The dashboard is at
**CloudWatch > Dashboards > `<stack name>-guardrail`**.

The agent prints the model id at startup and records the guardrail's id, version and
where it came from in the session record of the mission log; a session started with
`--no-guardrail` records `{"disabled": true}` and logs a warning.

**Delete** it (Cleanup in the [README](../README.md)):

```bash
aws cloudformation delete-stack --region us-west-2 --stack-name spot-demo-bedrock-guardrail
```

Every resource has `DeletionPolicy: Delete`. Delete this stack last: with the guardrail
gone, a session started without `--no-guardrail` fails at its first model call.

## Cost guardrails (`cost-guardrails.yaml`)

Optional sample infrastructure: the demo runs without this stack, but an account with no
spend controls of its own should have it.

The host bills while it runs, and its 200 GB volume bills while it is stopped: a
g6.4xlarge in us-west-2 is $1.3232 an hour On-Demand (Linux), about $966 for a 730-hour
month left on, and gp3 storage is $0.08 per GB-month, about $16 a month for the volume
(Amazon EC2 On-Demand list prices from the AWS Price List API, checked 2026-11-12;
re-check them, and the [README](../README.md)'s cost table, before quoting a number).
Forgetting to stop it is the main cost risk, and a start can fail for lack of capacity (threat model T39). This stack watches
the host and the account. It never creates, changes or deletes the instance itself.
Two related controls are not in the stack: the host-side idle stop,
`host/idle_stop.sh` ([RUNBOOK](../docs/RUNBOOK.md), "Once per host"), and a Capacity
Reservation for a scheduled demo ([RUNBOOK](../docs/RUNBOOK.md), "Capacity for a
scheduled demo").

**What it creates**

| Resource | What it does |
|---|---|
| `MonthlyCostBudget` (AWS Budgets) | A monthly COST budget on the **whole account**, emailing `NotificationEmail` at 80 % and 100 % of actual spend and at 100 % of forecast spend. It has no cost filter on purpose: the agent calls Amazon Bedrock through a system-defined cross-Region inference profile, and that usage carries no resource tags (only an application inference profile, a Bedrock Project or IAM principal attribution would add them); cost allocation tags are not retroactive; and the volume, the NAT gateway and the secret bill outside the instance. So deploy the demo in a dedicated account, where the account's cost is the demo's cost. Credits and refunds are excluded, so the budget tracks what the resources cost even in an account with credits. |
| `IdleStopAlarm` (CloudWatch) | Stops the instance with the EC2 stop action (`arn:aws:automate:<region>:ec2:stop`) when its average `CPUUtilization` stays below `IdleCpuPercent` for `IdlePeriods` consecutive 5-minute periods (basic monitoring; default 5 % for 2 hours), and emails the topic. It needs nothing on the host, so it also covers a host where `idle_stop.sh` was never applied or cannot tell. |
| `InstanceCheckRebootAlarm` (CloudWatch) | Reboots the instance (EC2 reboot action) after 3 consecutive minutes of a failed instance status check, and emails the topic. |
| `SystemCheckAlarm` (CloudWatch) | Emails the topic after 2 consecutive minutes of a failed system status check. With `SystemCheckAction=recover` it also takes the EC2 recover action, but only use that for an instance type that supports it: the g6 family supports neither CloudWatch action based recovery nor simplified automatic recovery (the EC2 User Guide's lists of supported types do not include G6, and both mechanisms exclude instances with instance store volumes, which g6.4xlarge has). On g6, stop and start the instance, which moves it to new hardware (`scripts/start-host.sh`). |
| `AlarmTopic`, `AlarmEmail`, `AlarmTopicPolicy` (SNS) | The alarms' email notifications. The topic policy lets only this account's alarms whose names start with the stack name publish. Confirm the subscription email that SNS sends first. |
| `AlarmTopicKey` (KMS) | A customer managed key, with rotation, that encrypts the topic. CloudWatch cannot publish to a topic encrypted with the AWS managed `alias/aws/sns` key; the key policy lets only this stack's alarms (by account and alarm-name prefix) use the key. It costs $1 a month, plus $1 a month for each of its first two yearly rotations (AWS KMS pricing). |

The budget itself is free (budgets without actions cost nothing); CloudWatch bills each of
the three standard alarms a small monthly fee (CloudWatch pricing).

**Parameters**

| Parameter | Default | Meaning |
|---|---|---|
| `InstanceId` | (required) | The sim host, in the stack's Region. |
| `MonthlyBudgetUSD` | (required) | The monthly budget for the whole account, in USD. Size it to your expected demo hours. |
| `NotificationEmail` | (required) | One address, for the budget alerts and the alarm notifications. |
| `IdleCpuPercent` | 5 | A 5-minute period counts as idle below this average CPU. Not measured for this demo: look at the instance's `CPUUtilization` graph for an idle host and for a running sim, and choose a value between them. |
| `IdlePeriods` | 24 | Consecutive idle 5-minute periods before the stop (24 is 2 hours; at most 288, the CloudWatch limit of one day for periods under an hour). Keep it longer than the host-side idle stop (default 60 minutes). |
| `SystemCheckAction` | `notify` | `notify`, or `recover` for an instance type whose `AutoRecoverySupported` is true (`aws ec2 describe-instance-types --instance-types <type> --query 'InstanceTypes[].AutoRecoverySupported'`). |

**Deploy** it from the Mac, as an administrator, in the host's Region:

```bash
aws cloudformation deploy --region us-west-2 --stack-name spot-demo-guardrails \
  --template-file infra/cost-guardrails.yaml \
  --parameter-overrides InstanceId=$SPOT_SIM_INSTANCE MonthlyBudgetUSD=<usd> \
    NotificationEmail=<you@example.com>
```

The deployer needs CloudFormation, AWS Budgets, CloudWatch, SNS and KMS permissions,
and `iam:CreateServiceLinkedRole`: alarms with EC2 actions use the service-linked role
`AWSServiceRoleForCloudWatchEvents`, which CloudWatch creates with the first such alarm
(or create it yourself with
`aws iam create-service-linked-role --aws-service-name events.amazonaws.com`). Then
confirm the SNS subscription email.

**Check** it:

```bash
aws cloudformation describe-stacks --region us-west-2 --stack-name spot-demo-guardrails \
  --query 'Stacks[0].[StackStatus,Outputs]'
aws cloudwatch describe-alarms --region us-west-2 --alarm-name-prefix spot-demo-guardrails- \
  --query 'MetricAlarms[].[AlarmName,StateValue,AlarmActions]'
aws budgets describe-budget --account-id <account-id> --budget-name spot-demo-guardrails-monthly-cost
aws sns list-subscriptions-by-topic --region us-west-2 --topic-arn <AlarmTopicArn output>
```

What to expect: alarms start in `INSUFFICIENT_DATA` and the status-check alarms move to
`OK` within minutes of the instance running; while the instance is stopped they have no
data (missing data is treated as missing, as the CloudWatch guide recommends for alarms
with EC2 actions). Budget data is refreshed up to three times a day, so the budget is
not a fast stop; and forecast alerts start only once the account has about five weeks
of usage. An alarm that stops or reboots the instance does not restart the demo: after
a reboot, re-apply `host/imds_guard.sh` and start the containers again.

**Delete** it (Cleanup in the [README](../README.md)):

```bash
aws cloudformation delete-stack --region us-west-2 --stack-name spot-demo-guardrails
aws cloudformation wait stack-delete-complete --region us-west-2 --stack-name spot-demo-guardrails
```

Every resource has `DeletionPolicy: Delete`: the budget, the alarms, the topic and its
subscription go with the stack, and the key is scheduled for deletion after 7 days
(cancel that with `aws kms cancel-key-deletion` if you need it back). The instance is
never part of the stack and is not touched. The service-linked role stays; delete it
only if no other alarm in the account uses an EC2 action.

## Session logging (`session-logging.yaml`)

Every operator connects through Session Manager as their own OS account, and Session
Manager streams each shell session to CloudWatch Logs as it happens (threat model T38;
the steps are in [`docs/iam/README.md`](../docs/iam/README.md)). The operator console
prints the mission log's chain head (`[audit head]` lines) into those sessions, so this
log group holds an off-host copy of the heads that host users cannot rewrite, and
`python3 agent/mission.py --verify` checks a mission log against it
([RUNBOOK](../docs/RUNBOOK.md), "Check a log against the session log").

**What it creates**

| Resource | What it does |
|---|---|
| `SessionLogKey` (KMS) | A symmetric customer managed key with rotation. Its key policy lets the account's IAM policies govern its use, and lets CloudWatch Logs in this Region use it only for this log group (a `StringEquals` condition on the `aws:logs:arn` encryption context, the operator IAM Access Analyzer expects for that key). It costs $1 a month, plus $1 a month for each of its first two yearly rotations (AWS KMS pricing). |
| `SessionLogKeyAlias` (KMS) | An alias for the key. |
| `SessionLogGroup` (CloudWatch Logs) | The log group Session Manager streams to, encrypted with the key, kept for `RetentionInDays`. CloudWatch Logs bills ingestion and storage (CloudWatch pricing); a demo session is small. |

**Parameters**

| Parameter | Default | Meaning |
|---|---|---|
| `LogGroupName` | `/spot-demo/session-manager` | The log group. `docs/iam/session-preferences.json` and `docs/iam/instance-role-policy.json` name the default: change all three together. |
| `RetentionInDays` | 365 | How long CloudWatch Logs keeps the sessions; one year or more. |

**Deploy** it from the Mac, as an administrator, in the host's Region, before the
preferences (a session whose log group is missing or unencrypted does not start once
the preferences require encryption):

```bash
aws cloudformation deploy --region us-west-2 --stack-name spot-demo-session-logging \
  --template-file infra/session-logging.yaml
aws cloudformation describe-stacks --region us-west-2 --stack-name spot-demo-session-logging \
  --query 'Stacks[0].Outputs'      # KeyId replaces <SESSION_LOG_KEY_ID> in the instance role policy
```

Then add the session-log statements of `docs/iam/instance-role-policy.json` to the
instance role, tag the operators, create their OS accounts and apply the preferences, in
the order given in [`docs/iam/README.md`](../docs/iam/README.md).

**Check** it:

```bash
aws logs describe-log-groups --region us-west-2 --log-group-name-prefix /spot-demo/session-manager \
  --query 'logGroups[].[logGroupName,kmsKeyId,retentionInDays]'
```

After an operator's first session, a log stream named after the session appears in the
group within a minute or two.

**Delete** it (Cleanup in the [README](../README.md)): the key and the log group have
`DeletionPolicy: Retain`, because they hold the audit trail, so `delete-stack` removes
the alias only and leaves both:

```bash
aws cloudformation delete-stack --region us-west-2 --stack-name spot-demo-session-logging
# once you no longer need the sessions:
aws logs delete-log-group --region us-west-2 --log-group-name /spot-demo/session-manager
aws kms schedule-key-deletion --region us-west-2 --key-id <KeyId output> --pending-window-in-days 7
```

Roll the preferences back first ([`docs/iam/README.md`](../docs/iam/README.md)), or new
sessions fail while they still require the deleted log group.
