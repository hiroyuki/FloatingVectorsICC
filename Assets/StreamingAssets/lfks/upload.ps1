<#
LFKS token upload script (Windows PowerShell 5.1+ / pwsh 7+).

Usage — download once, then run (the downloaded copy already knows the
server it came from, so no -Url is needed):

  irm https://<worker-domain>/upload.ps1 -OutFile upload.ps1
  .\upload.ps1 -Token <TOKEN> -File C:\path\to\file

Optional extras: -Directory sub (subfolder placement), -Name stored.bin
(store under a different name) and -Url https://<worker-domain> (override
the server).

If you prefer to run it in one go without saving a file (the PowerShell
equivalent of curl | sh; scriptblock::Create is used instead of iex "..."
because iex with a double-quoted string would expand $variables inside the
fetched script):

  & ([scriptblock]::Create((irm https://<worker-domain>/upload.ps1))) -Token <TOKEN> -File C:\path\to\file

Parameters:
  -Token      upload token you were given (required)
  -File       file to upload (required)
  -Directory  subfolder of the granted directory to upload into, e.g. "sub"
              or "a/b" (optional; default is the directory root)
  -Name       store the file under this name instead of the local one
              (optional)
  -Json       suppress progress output; print only the final JSON result
              line (errors still go to stderr)
  -Url        worker origin (defaults to the host this script was
              downloaded from)

Files up to 50MB go through the direct lane (a single PUT; the file is
distributable the moment it returns). Larger files are sent in 50MB parts
(matching the browser uploader's CHUNK_SIZE) with up to 3 attempts per part.
Progress goes to stderr; on success stdout carries exactly one
machine-readable JSON line {id, name, relativePath, size, downloadUrl} for
pipelines.

Exit codes: 1 usage error, 2 file not readable, 3 start failed,
            4 upload failed, 5 complete failed.
#>
param(
	[Parameter(Mandatory = $true)][string]$Token,
	[Parameter(Mandatory = $true)][string]$File,
	[string]$Directory = "",
	[string]$Name = "",
	[switch]$Json,
	# Replaced with the serving origin by the GET /upload.ps1 route handler, so
	# the URL the script was fetched from is also its default API endpoint.
	[string]$Url = "https://ntticc.lfks.app"
)

$ErrorActionPreference = "Stop"
$ChunkSize = 50MB
if ($env:UPLOAD_CHUNK_SIZE) { $ChunkSize = [long]$env:UPLOAD_CHUNK_SIZE } # test override
$MaxRetries = 3

function Write-Progress-Line([string]$Message) {
	if ($Json) { return }
	[Console]::Error.WriteLine($Message)
}

function Fail([int]$Code, [string]$Message) {
	[Console]::Error.WriteLine("error: $Message")
	exit $Code
}

# Windows PowerShell 5.1 may still default to pre-TLS1.2 protocols.
try {
	[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
} catch {}

# When the script is run straight from the repository the placeholder default
# was never substituted; it does not look like a URL, so require -Url then.
if ($Url -notmatch '^https?://') {
	Fail 1 "-Url is required when the script was not fetched from the worker"
}
if (-not (Test-Path -LiteralPath $File -PathType Leaf)) {
	Fail 2 "file not found or not readable: $File"
}

$FileItem = Get-Item -LiteralPath $File
$Size = [long]$FileItem.Length
if ($Size -le 0) { Fail 2 "file is empty: $File" }
$LocalName = $FileItem.Name
# The server stores StoredName (display name and, with -Directory, path); the
# true LocalName still goes into the start body for extension resolution.
$StoredName = if ($Name -ne "") { $Name } else { $LocalName }
$RelPath = if ($Directory -ne "") { "$Directory/$StoredName" } elseif ($Name -ne "") { $StoredName } else { "" }
$TotalParts = [int][math]::Ceiling($Size / [double]$ChunkSize)

# request helper: returns @{ ok; data } on 2xx, @{ ok = $false; error; status }
# with the server's error body otherwise (5.1 has no -SkipHttpErrorCheck).
function Invoke-Api {
	param([string]$Method, [string]$Uri, $Body, [string]$ContentType)
	try {
		if ($null -ne $Body) {
			return @{ ok = $true; data = Invoke-RestMethod -Method $Method -Uri $Uri -Body $Body -ContentType $ContentType }
		}
		return @{ ok = $true; data = Invoke-RestMethod -Method $Method -Uri $Uri }
	} catch {
		$message = $null
		$status = 0
		if ($_.Exception.Response) {
			try { $status = [int]$_.Exception.Response.StatusCode } catch {}
		}
		if ($_.ErrorDetails -and $_.ErrorDetails.Message) {
			$message = $_.ErrorDetails.Message
		} elseif ($_.Exception.Response) {
			try {
				$reader = New-Object System.IO.StreamReader($_.Exception.Response.GetResponseStream())
				$message = $reader.ReadToEnd()
			} catch {}
		}
		if (-not $message) { $message = $_.Exception.Message }
		return @{ ok = $false; error = $message; status = $status }
	}
}

function Invoke-JsonPost([string]$Uri, [hashtable]$BodyObject) {
	$json = $BodyObject | ConvertTo-Json -Compress
	# send explicit UTF-8 bytes: 5.1 would otherwise mangle non-ASCII filenames
	return Invoke-Api -Method Post -Uri $Uri -Body ([Text.Encoding]::UTF8.GetBytes($json)) -ContentType "application/json; charset=utf-8"
}

# The server-side path (relative to the token's directory) both lanes store.
$UploadPath = if ($RelPath -ne "") { $RelPath } else { $StoredName }
$FileId = $null
$FsPath = $null

# --- direct lane: files up to ChunkSize go in a single PUT --------------------
# The file is distributable at /fs/ the moment the request returns. A 413
# (size boundary disagreement with the server) falls back to the part flow.
if ($Size -le $ChunkSize) {
	Write-Progress-Line "uploading $LocalName as $UploadPath ($Size bytes, single request)"
	$bytes = [System.IO.File]::ReadAllBytes($FileItem.FullName)
	$encodedPath = ($UploadPath -split "/" | ForEach-Object { [uri]::EscapeDataString($_) }) -join "/"
	$attempt = 1
	while ($true) {
		$res = Invoke-Api -Method Put -Uri "$Url/api/upload?token=$Token&path=$encodedPath" -Body $bytes -ContentType "application/octet-stream"
		if ($res.ok) {
			$FileId = $res.data.id
			if (-not $FileId) { Fail 4 "upload failed: unexpected response" }
			if ($res.data.fsPath) { $FsPath = [string]$res.data.fsPath }
			break
		}
		if ($res.status -eq 413) {
			Write-Progress-Line "server declined the single request (HTTP 413), falling back to parts"
			break
		}
		if ($res.status -in 400, 403, 404) { Fail 4 "upload failed (HTTP $($res.status)): $($res.error)" }
		if ($attempt -ge $MaxRetries) {
			Fail 4 "upload failed after $MaxRetries attempts: $($res.error)"
		}
		Write-Progress-Line "attempt $attempt failed, retrying..."
		Start-Sleep -Seconds $attempt
		$attempt++
	}
}

# Stream.Read may return fewer bytes than asked even mid-file; R2 requires all
# non-trailing parts to be exactly the same length, so fill the buffer fully.
function Read-Chunk([System.IO.Stream]$Stream, [byte[]]$Buffer) {
	$offset = 0
	while ($offset -lt $Buffer.Length) {
		$n = $Stream.Read($Buffer, $offset, $Buffer.Length - $offset)
		if ($n -le 0) { break }
		$offset += $n
	}
	return $offset
}

# --- multipart lane: everything else (and a 413 fallback) ---------------------
if (-not $FileId) {
	Write-Progress-Line "uploading $LocalName as $UploadPath ($Size bytes, $TotalParts part(s))"

	# --- 1. start a session ----------------------------------------------------
	$startBody = @{ filename = $LocalName }
	if ($RelPath -ne "") { $startBody.relativePath = $RelPath }

	$start = Invoke-JsonPost "$Url/api/upload?token=$Token" $startBody
	if (-not $start.ok) { Fail 3 "could not start the upload: $($start.error)" }
	$SessionId = $start.data.uploadSessionId
	if (-not $SessionId) { Fail 3 "could not start the upload: unexpected response" }

	# --- 2. upload each part with retries --------------------------------------
	$stream = [System.IO.File]::OpenRead($FileItem.FullName)
	try {
		$buffer = New-Object byte[] $ChunkSize
		for ($part = 1; $part -le $TotalParts; $part++) {
			Write-Progress-Line "part $part/$TotalParts ..."
			$read = Read-Chunk $stream $buffer
			if ($read -le 0) { Fail 2 "could not read part $part from $File" }
			if ($read -eq $buffer.Length) {
				$chunk = $buffer
			} else {
				# NB: $buffer[0..n] would build an object[], not a byte[]
				$chunk = New-Object byte[] $read
				[Array]::Copy($buffer, $chunk, $read)
			}

			$attempt = 1
			while ($true) {
				$res = Invoke-Api -Method Put -Uri "$Url/api/upload?token=$Token&session=$SessionId&partNumber=$part" -Body $chunk -ContentType "application/octet-stream"
				if ($res.ok) { break }
				if ($attempt -ge $MaxRetries) {
					Fail 4 "part $part/$TotalParts failed after $MaxRetries attempts: $($res.error)"
				}
				Write-Progress-Line "part $part attempt $attempt failed, retrying..."
				Start-Sleep -Seconds $attempt
				$attempt++
			}
		}
	} finally {
		$stream.Dispose()
	}

	# --- 3. complete -----------------------------------------------------------
	$complete = Invoke-JsonPost "$Url/api/upload?token=$Token&session=$SessionId" @{ originalFilename = $StoredName }
	if (-not $complete.ok) { Fail 5 "could not complete the upload: $($complete.error)" }
	$FileId = $complete.data.id
	if (-not $FileId) { Fail 5 "could not complete the upload: unexpected response" }
	if ($complete.data.fsPath) { $FsPath = [string]$complete.data.fsPath }
}

# The full server-side path (directory name included) for the public /fs/ route.
if (-not $FsPath) { $FsPath = $StoredName }
$FsUrl = "$Url/fs/" + (($FsPath -split "/" | ForEach-Object { [uri]::EscapeDataString($_) }) -join "/")

# Same download URL the browser file list builds: /files/{id}{ext}, extension
# mirroring node's extname (dotfiles like ".env" count as extensionless).
$Ext = [System.IO.Path]::GetExtension($StoredName)
if ($StoredName.StartsWith(".") -and $StoredName.LastIndexOf(".") -eq 0) { $Ext = "" }
$DownloadUrl = "$Url/files/$FileId$Ext"

Write-Progress-Line "done: $StoredName uploaded (file id: $FileId)"
Write-Progress-Line "download: $DownloadUrl"
Write-Progress-Line "fs: $FsUrl"
# stdout carries exactly one machine-readable JSON line for pipelines;
# relativePath is in this route's coordinates, i.e. relative to the token's dir.
$rel = if ($RelPath -ne "") { $RelPath } else { $StoredName }
[PSCustomObject]@{
	id           = $FileId
	name         = $StoredName
	relativePath = $rel
	size         = $Size
	downloadUrl  = $DownloadUrl
	fs           = $FsUrl
} | ConvertTo-Json -Compress
