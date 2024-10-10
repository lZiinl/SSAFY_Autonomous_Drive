/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './index.html',  // Vite의 index.html 경로
    './src/**/*.{js,jsx,ts,tsx}',  // React 소스 경로
  ],
  theme: {
    extend: {},
  },
  plugins: [],
}