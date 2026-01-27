# Fork Workflow Guide

This document describes the branching strategy and workflow for maintaining our QGroundControl fork while staying in sync with upstream updates.

## Branch Structure

| Branch | Purpose | Tracks |
|--------|---------|--------|
| `upstream-master` | Clean copy of upstream - never commit here directly | `upstream/master` |
| `master` | Organization's stable/release branch | `origin/master` |
| `develop` | Active development with customizations | `origin/develop` |

## Remotes

| Remote | Repository |
|--------|------------|
| `origin` | Our fork (HIGHCATOFFICIAL/qgroundcontrol) |
| `upstream` | Original repo (mavlink/qgroundcontrol) |

## Making Customizations

1. Create a feature branch from `develop`:
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b feature/your-feature-name
   ```

2. Make your changes and commit:
   ```bash
   git add <files>
   git commit -m "Description of changes"
   ```

3. Push and create a pull request:
   ```bash
   git push -u origin feature/your-feature-name
   ```
   Then create a PR to merge into `develop`.

## Syncing with Upstream Updates

### Regular Sync

1. Fetch latest from upstream:
   ```bash
   git fetch upstream
   ```

2. Update the `upstream-master` tracking branch:
   ```bash
   git checkout upstream-master
   git merge upstream/master
   git push origin upstream-master
   ```

3. Merge upstream changes into `develop`:
   ```bash
   git checkout develop
   git merge upstream-master
   # Resolve any conflicts with your customizations
   git push origin develop
   ```

### Releasing to Master

When `develop` is stable and ready for release:
```bash
git checkout master
git merge develop
git push origin master
```

## Handling Specific Scenarios

### Cherry-picking Critical Fixes

For urgent upstream fixes that need immediate integration:
```bash
git checkout develop
git cherry-pick <commit-hash>
git push origin develop
```

### Resolving Merge Conflicts

When merging upstream changes, conflicts may occur in files you've customized. To resolve:

1. Open conflicting files and look for conflict markers (`<<<<<<<`, `=======`, `>>>>>>>`)
2. Keep your customizations where appropriate, incorporate upstream changes where needed
3. Test thoroughly after resolving
4. Commit the merge:
   ```bash
   git add <resolved-files>
   git commit
   ```

### Viewing Upstream Changes Before Merging

To see what's new in upstream before merging:
```bash
git fetch upstream
git log upstream-master..upstream/master --oneline
git diff upstream-master..upstream/master
```

## Best Practices

- Never commit directly to `upstream-master` - it should always mirror upstream
- Keep customizations minimal and well-documented to reduce merge conflicts
- Sync with upstream regularly to avoid large, complex merges
- Tag releases on `master` for easy rollback if needed
- Document any files heavily modified to watch during upstream merges
