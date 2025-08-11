import React from 'react'
import ReactDOM from 'react-dom/client'
import { CssBaseline, ThemeProvider, createTheme } from '@mui/material'
import App from './ui/App'

const theme = createTheme({
  palette: {
    mode: 'light',
    primary: { main: '#0ea5e9' },
    secondary: { main: '#22c55e' },
  },
  shape: { borderRadius: 12 },
})

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <App />
    </ThemeProvider>
  </React.StrictMode>
)



