param(
    [string]$Mode,
    [Parameter(ValueFromRemainingArguments=$true)][string[]]$Files
)

function Get-NormalizedSha1 {
    param([byte[]]$Bytes)
    # Strip CR so CRLF and LF line endings hash identically to a Linux checkout.
    $normalized = New-Object System.Collections.Generic.List[byte]
    foreach ($b in $Bytes) {
        if ($b -ne 13) { $normalized.Add($b) }
    }
    $sha1 = [System.Security.Cryptography.SHA1]::Create()
    $hash = $sha1.ComputeHash($normalized.ToArray())
    return -join ($hash | ForEach-Object { $_.ToString("x2") })
}

$lines = foreach ($f in $Files) {
    $bytes = [System.IO.File]::ReadAllBytes($f)
    $h = Get-NormalizedSha1 -Bytes $bytes
    "$h  $f"
}

# Build the combined listing with LF-only line endings, matching what
# `shasum file1 file2 ... | shasum` would produce on Linux.
$combinedText = ($lines -join "`n") + "`n"
$combinedBytes = [System.Text.Encoding]::UTF8.GetBytes($combinedText)
$combined = Get-NormalizedSha1 -Bytes $combinedBytes

if ($Mode -eq "combined") {
    Write-Output $combined
} else {
    Write-Output $combined
    $lines | ForEach-Object { Write-Output $_ }
}