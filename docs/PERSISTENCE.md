# Persistence System Documentation

## Overview

The Dino Core Robot Control System now includes a comprehensive persistence system that automatically saves and restores user preferences, working state, and system configurations. When you reopen the system, all your latest work will be available immediately.

## Features

### 🔄 Auto-Save Functionality

- **Real-time saving**: Changes are automatically saved within 1-2 seconds
- **Smart debouncing**: Prevents excessive saves while typing or making rapid changes
- **Background operation**: No interruption to workflow

### 💾 What Gets Saved

#### Terminal System

- Active tabs and their configurations
- Command history for each terminal type (Bash, ROS, Python, SSH)
- Current working directories
- Terminal preferences (theme, font size)

#### User Settings

- Interface preferences (theme, language, font size)
- Display options (grid, tooltips, compact mode)
- Notification settings
- Robot control preferences
- Performance configurations
- Advanced developer settings

#### System Monitoring

- Auto-refresh settings
- Chart preferences
- Monitoring view configurations

#### API Management

- Selected API category
- Test endpoint configurations
- Request payloads and responses
- API call history (last 50 calls)

#### Python Development

- Active code examples
- Tab preferences
- Development environment settings

### 🛡️ Data Security & Privacy

- **Local storage only**: All data stays on your device
- **No server transmission**: Nothing is sent to external servers
- **Browser-based**: Uses standard localStorage API
- **User control**: Complete control over data export/import/deletion

## Usage

### Automatic Operation

The persistence system works automatically - no setup required. Just use the system normally and your preferences will be saved.

### Manual Controls

#### Persistence Status Indicator

- Located in the top-right header
- Shows current storage usage
- Displays last save time
- Provides storage management options

#### Settings > Data Management Tab

- View what's being saved
- Export all data to JSON file
- Import previously exported data
- Clear all saved data

### Data Export/Import

```bash
# Export data
Click "Export All Data" in Settings > Data Management

# Import data
Click "Import Data" and select your exported JSON file
```

## Technical Implementation

### Core Components

#### PersistenceService

- Singleton service managing localStorage operations
- Handles TTL (time-to-live) for data expiration
- Provides subscription system for real-time updates
- Error handling for storage limitations

#### PersistenceContext

- React context for easy access across components
- Namespaced storage with app prefix
- Loading state management

#### Custom Hooks

- `usePersistentState`: For single values with auto-save
- `usePersistentStore`: For complex objects with field updates
- `usePersistentArray`: For arrays with built-in management

### Storage Structure

```javascript
{
  "dino-core-user-settings": {
    value: { theme: "dark", language: "th", ... },
    timestamp: 1703123456789,
    ttl: null
  },
  "dino-core-terminal-tabs": {
    value: [{ id: "1", name: "bash", ... }],
    timestamp: 1703123456789,
    ttl: null
  }
  // ... other namespaced data
}
```

### Error Handling

- Graceful fallback to default values
- Storage quota exceeded warnings
- Data corruption recovery
- TTL expiration cleanup

## Storage Limits

### Browser Limits

- **localStorage**: ~5-10MB per domain (browser dependent)
- **Storage monitoring**: Real-time usage tracking
- **Cleanup**: Automatic removal of expired data

### Optimization

- Efficient serialization
- Data compression for large objects
- Automatic cleanup of old entries
- Storage usage warnings

## Troubleshooting

### Storage Full

1. Check storage usage in persistence status indicator
2. Clear old data in Settings > Data Management
3. Export important data before clearing

### Data Not Saving

1. Check browser localStorage permissions
2. Verify not in incognito/private mode
3. Check browser storage settings

### Lost Data

1. Check if data was exported previously
2. Look for browser backup/sync features
3. Data may be cleared by browser cleanup tools

## Development

### Adding Persistence to New Components

```typescript
// Simple state persistence
const { state, setState, isLoading } = usePersistentState({
  key: "my-component-state",
  defaultValue: "initial value",
  autoSave: true,
  autoSaveDelay: 1000,
});

// Complex object persistence
const { store, updateField, saveNow } = usePersistentStore(
  "my-component-store",
  {
    setting1: "default",
    setting2: false,
    setting3: 100,
  },
);

// Array persistence with management
const { items, addItem, removeItem, clearItems } = usePersistentArray(
  "my-component-array",
  50, // max items
);
```

### Best Practices

- Use meaningful key names
- Set appropriate TTL for temporary data
- Handle loading states
- Provide fallback defaults
- Test with storage disabled

## Migration & Updates

### Data Migration

The system includes automatic migration for data structure changes:

- Version detection
- Backward compatibility
- Graceful upgrades

### Future Enhancements

- Cloud synchronization option
- Encrypted storage for sensitive data
- Advanced data compression
- Cross-device synchronization

---

_This persistence system ensures a seamless, continuous experience where your work is never lost and the system remembers exactly how you left it._
