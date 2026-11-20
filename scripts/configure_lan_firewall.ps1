$ErrorActionPreference = "Stop"

if (-not (Get-NetFirewallRule -DisplayName "Smart Trolley Dashboard LAN" -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName "Smart Trolley Dashboard LAN" `
        -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8000 `
        -RemoteAddress LocalSubnet -Profile Public,Private | Out-Null
}

if (-not (Get-NetFirewallRule -DisplayName "Smart Trolley MQTT LAN" -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName "Smart Trolley MQTT LAN" `
        -Direction Inbound -Action Allow -Protocol TCP -LocalPort 1883 `
        -RemoteAddress LocalSubnet -Profile Public,Private | Out-Null
}

Write-Host "Smart Trolley LAN firewall rules are ready."
