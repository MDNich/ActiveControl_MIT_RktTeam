# OpenRocket MIT 6.3 error reporting

The MIT edition's report destination is **therobomentors@gmail.com**. No server or email credentials are needed for the current workflow.

## Current behavior

Use **Help → Bug report**, or **View bug report** after an exception. Both open the same report editor with the MIT maintainer's address. The upstream GitHub link and mailing-list address are absent from the MIT dialog.

- **Copy report** copies the edited report as plain text.
- **Save report…** writes the edited report to a UTF-8 text file selected by the user.
- **Copy & email** copies the report and opens the user's mail application with the recipient and a versioned subject already filled in. The user pastes the report, or attaches the saved text file, and sends the message. If no mail application can be opened, the dialog explains how to email the copied report manually.

The complete report is kept out of the `mailto:` URL, avoiding mail-client URL-length limits. Nothing is sent in the background. The legacy `BugReporter.sendBugReport` HTTP entry point refuses to connect in MIT builds, including when a debug URL override is supplied. There is no fallback to OpenRocket's reporting server.

The recipient is configured once in `core/src/main/resources/build.properties`:

```properties
build.mit.bugreport.email=therobomentors@gmail.com
```

Reports include the MIT version, upstream version, system/JVM details and buffered application logs. Exception reports also include the stack trace and thread information. Users can edit the report before sharing it. The rocket `.ork` file is not attached automatically; the report asks the user to include it when useful.

## If automatic in-app submission is wanted later

An inbox address alone does not provide a service that the app can POST to. A direct in-app **Send report** action would need an owned HTTPS receiving service and a server-side email sender.

Provide or choose:

1. **An HTTPS endpoint and hosting account** for receiving reports. A small hosted function or web service is sufficient; no dedicated machine is required.
2. **An email sending account and sender address**, with its API credentials, SMTP credentials, or OAuth access configured on the server. The recipient remains `therobomentors@gmail.com`. Credentials must not be embedded in the distributed JAR or desktop bundles.
3. **A delivery/storage choice:** email only, or also retain reports in server storage, with a retention period.

The service should enforce a report-size limit and rate limits, and keep the recipient fixed server-side so it cannot become an arbitrary email relay. It should return success only after accepting the report into durable storage or a delivery queue, with useful errors for rejected requests.

We can reuse the legacy form-POST shape (`version` and `content`, UTF-8) and HTTP 202 response, while supplying the MIT version and targeting only the owned HTTPS endpoint. That would require a follow-up change to enable the MIT transport and connect a reviewed-report submission button to it. It is deliberately disabled in the current build.

No live report emails were sent during verification; transport tests use mock connections.
