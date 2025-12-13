/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        emerald: {
          50: '#f0fdf4',
          100: '#dcfce7',
          200: '#bbf7d0',
          300: '#86efac',
          400: '#4ade80',
          500: '#22c55e',
          600: '#16a34a',
          700: '#15803d',
          800: '#166534',
          900: '#145231',
          950: '#052e16',
        },
        teal: {
          50: '#f0fdfa',
          100: '#ccfbf1',
          200: '#99f6e4',
          300: '#5eead4',
          400: '#2dd4bf',
          500: '#14b8a6',
          600: '#0d9488',
          700: '#0f766e',
          800: '#115e59',
          900: '#134e4a',
          950: '#0d3331',
        },
      },
    },
  },
  plugins: [],
  safelist: [
    // Emerald gradients
    'from-emerald-400',
    'from-emerald-500',
    'from-emerald-600',
    'from-emerald-700',
    'to-emerald-600',
    'to-emerald-700',
    'to-emerald-800',
    'to-emerald-900',
    'to-teal-600',
    'to-teal-400',
    'to-green-800',
    // Hover effects
    'hover:from-emerald-300',
    'hover:to-teal-300',
    // Text colors
    'text-emerald-50',
    'text-emerald-100',
    'text-emerald-400',
    'text-emerald-500',
    'text-slate-300',
    'text-slate-400',
    // Background colors
    'bg-emerald-500',
    'bg-emerald-600',
    'bg-emerald-700',
    'bg-emerald-900',
    'bg-slate-800',
    'bg-slate-900',
    // Border colors
    'border-emerald-500',
    'border-emerald-700',
    'border-teal-500',
    'border-white',
  ],
};
