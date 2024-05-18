# Use the official lightweight Node.js
FROM node:alpine

# Set working directory
WORKDIR /app
# add `/app/node_modules/.bin` to $PATH
ENV PATH /app/node_modules/.bin:$PATH

# Copy package.json and yarn.lock to install dependencies
COPY package.json yarn.lock ./

# Install app dependencies
RUN yarn install

# Add app files
COPY . .

# Expose port 3000
EXPOSE 3000

# Start the app
CMD ["yarn", "start"]
